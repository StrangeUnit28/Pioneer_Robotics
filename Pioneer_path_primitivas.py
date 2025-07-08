import math
import sim
import numpy as np
import math as mat
import time
import csv

class Pioneer():
    """"
    Corobeu class is the principal class to controller the EVA positions robots.
        __init___                   = Function to inicializate the global variables
        connect_Pioneer             = Present the connection with the coppeliaSim
        Speed_Pioneer               = Calculated the wheels speeds based on the robot topology (in this case is a differential robot)
        PID_Controller_phi          = Implement the PID based on the phi error
        Robot_Pioneer               = The principal function in this class.
    """""

    def __init__(self):

        """"
            Initializations global variables
            self.y_out    (float list) = Out position robot in the y axis
            self.x_out    (Float list) = Out position robot in the x axis
            self.phi      (Float)   = phi robot value
            self.v_max    (Integer) = max speed for the robot
            self.v_min    (Integer) = min speed for the robot
            self.posError (Float list) = phi error
        """""

        self.y_out = []
        self.x_out = []
        self.phi = 0
        self.v_max_wheels = 15
        self.v_min_wheels = -15
        self.v_linear = 0.1
        self.posError = []
        self.Min_error_distance = 0.1

    def connect_Pioneer(self, port):
        """""
        Function used to communicate with CoppeliaSim
            argument :
                Port (Integer) = used to CoppeliaSim (same CoppeliaSim)

            outputs : 
                clientID (Integer)  = Client number
                robot    (Integer)  = objecto robot
                MotorE   (Integer)  = Object motor left
                MotorD   (Integer)  = Object motor right
                ball     (Integer)  = Object ball on the scene
        """""

        ### Connect to coppeliaSim ###

        sim.simxFinish(-1)
        clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)
        if clientID == 0:
            print("Connect to", port)
        else:
            print("Can not connect to", port)

        ### Return the objects ###

        returnCode, robot = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx',
                                                    sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx" de v - rep(Robot Movil)
        returnCode, MotorE = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor',
                                                     sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_leftmotor" de v-rep (motor izquierdo)
        returnCode, MotorD = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor',
                                                     sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_rightMotor" de v - rep(motor derecho)
        
        ballHandler = [0, 0, 0 , 0]
        returnCode, ballHandler[0] = sim.simxGetObjectHandle(clientID, 'ball_1', sim.simx_opmode_blocking)
        returnCode, ballHandler[1] = sim.simxGetObjectHandle(clientID, 'ball_2', sim.simx_opmode_blocking)
        returnCode, ballHandler[2] = sim.simxGetObjectHandle(clientID, 'ball_3', sim.simx_opmode_blocking)
        returnCode, ballHandler[3] = sim.simxGetObjectHandle(clientID, 'ball_4', sim.simx_opmode_blocking)

        return clientID, robot, MotorE, MotorD, ballHandler
    
    '''
        Pionner's starting position
        robot = [-2.0, -2.0]

        Ball's positons
        ball_1 = [-2.0, +1.5]
        ball_2 = [-0.5, -0.5]
        ball_3 = [+1.5, +2.0]
        ball_4 = [+1.5, -1.5]
    '''

    def Speed_Pioneer(self, U, omega, lock_stop_simulation, signed, error_phi):

        """""
        Function used to calculate the speed for each wheel based on the topology robot
            argument :
                U              (Integer) = Max linear speed
                omega          (Float)   = Angular speed
                error_distance (Float)   = Error between the robot and the final point

            outputs : 
                vl (Float)   = Left speed
                vd (Float)   = right speed
                a  (Integer) = condition to simulate
        """""

        ### Calculate the speed based on the topology robot ###

        # L = 381  # Distance between the wheels
        # R = 95  # Wheel radio
        L = 0.381
        R = 0.095

        ### Calculate the speed based on the topology robot ###

        vd = ((2 * (U) + omega * L) / (2 * R))
        vl = ((2 * (U) - omega * L) / (2 * R))
        a = 1
        # print(f'vl === {vl} vd == {vd}')
        # print(f' omega == {omega}')
        ### the first time #####

        Max_Speed = self.v_max_wheels
        Min_Speed = self.v_min_wheels

        # Max_Speed = self.v_max
        # Min_Speed = self.v_min

        ### Saturation speed upper ###

        if vd >= Max_Speed:
            vd = Max_Speed
        if vd <= Min_Speed:
            vd = Min_Speed

        ### Saturation speed Lower ###

        if vl >= Max_Speed:
            vl = Max_Speed
        if vl <= Min_Speed:
            vl = Min_Speed

        ### When arrive to the goal ###

        if lock_stop_simulation == 1 and error_phi <= 0.08:
            a = 0
            vl = 0
            vd = 0

        ### Return values ###

        return vl, vd, a

    def Robot_Pioneer_Primitivas(self, primitiva, clientID, robot, motorE, motorD, ball):
        Number_Iterations = 0

        if sim.simxGetConnectionId(clientID) == -1:
            return False

        s, start_pos = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_blocking)
        s, start_ori = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_blocking)
        start_z_degrees = round(math.degrees(start_ori[2]), 2)

        while True:
            s, current_pos = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_blocking)
            s, current_ori = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_blocking)
            current_z_degrees = round(math.degrees(current_ori[2]), 2)

            dx = round(abs(current_pos[0] - start_pos[0]), 2)
            dy = round(abs(current_pos[1] - start_pos[1]), 2)
            d_phi = round(abs(current_z_degrees - start_z_degrees), 2)

            controller_Linear = self.v_linear
            lock_stop_simulation = 0

            if primitiva == "forward":
                if dx >= 0.49 or dy >= 0.49:
                    break
                omega = 0
                vl, vd, _ = self.Speed_Pioneer(controller_Linear, omega, lock_stop_simulation, 1, 0)

            elif primitiva == "right":
                if d_phi >= 89:
                    break
                radio_ideal = -0.5
                omega = self.v_linear / radio_ideal
                vl, vd, _ = self.Speed_Pioneer(controller_Linear, omega, lock_stop_simulation, 1, 0)

            elif primitiva == "left":
                if d_phi >= 89:
                    break
                radio_ideal = 0.5
                omega = self.v_linear / radio_ideal
                vl, vd, _ = self.Speed_Pioneer(controller_Linear, omega, lock_stop_simulation, 1, 0)

            sim.simxSetJointTargetVelocity(clientID, motorE, vl, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(clientID, motorD, vd, sim.simx_opmode_blocking)

            Number_Iterations += 1
            self.x_out.append(current_pos[0])
            self.y_out.append(current_pos[1])

        sim.simxSetJointTargetVelocity(clientID, motorE, 0, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(clientID, motorD, 0, sim.simx_opmode_blocking)

        return True
    
    def _decidir_movimento(self, angle_z, eixo, error_x, error_y):
        angle_z = round(angle_z)
        abs_angle = abs(angle_z)

        if eixo == 'x':
            if error_x > 0:
                if 0 <= abs_angle <= 5:
                    print("1")
                    return 'forward'
                elif 265 <= abs_angle <= 275:
                    print("2")
                    return 'left'
                elif 85 <= abs_angle <= 95:
                    if angle_z < 0:
                        print("21")
                        return 'left'
                    else:
                        print("3")
                        return 'right'
                elif 175 <= abs_angle <= 185:
                    print("4")
                    return 'left'
            else:
                if 175 <= abs_angle <= 185:
                    print("5")
                    return 'forward'
                elif 85 <= abs_angle <= 95:
                    print("6")
                    return 'left'
                elif 265 <= abs_angle <= 275:
                    print("7")
                    return 'right'
                elif 0 <= abs_angle <= 5:
                    print("8")
                    return 'left'

        elif eixo == 'y':
            if error_y > 0:
                if 85 <= abs_angle <= 95:
                    print("9")
                    return 'forward'
                elif 0 <= abs_angle <= 5:
                    print("10")
                    return 'left'
                elif 175 <= abs_angle <= 185:
                    print("11")
                    return 'right'
                elif 265 <= abs_angle <= 275:
                    print("12")
                    return 'left'
            else:
                if 265 <= abs_angle <= 275:
                    if angle_z < 0:
                        print("20")
                        return 'left'
                    else:
                        print("13")
                        return 'forward'
                elif 0 <= abs_angle <= 5:
                    # if angle_z < 0:
                    #     print("19")
                    #     return 'left'
                    # else:
                    print("14")
                    return 'right'
                elif 175 <= abs_angle <= 185:
                    if angle_z < 0:
                        print("18")
                        return 'right'
                    else:
                        print("15")
                        return 'left'
                elif 85 <= abs_angle <= 95:
                    if angle_z < 0:
                        print("17")
                        return 'forward'
                    else:
                        print("16")
                        return 'right'

        return 'forward'

    def executar_primitivas(self, filename):
        clientID, robot, motorE, motorD, ballHandler = self.connect_Pioneer(19999)

        if sim.simxGetConnectionId(clientID) == -1:
            print("Falha ao conectar com o CoppeliaSim.")
            return
        
        i = 0
        
        for ball in ballHandler:
            while True:
                _, position = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_blocking)
                _, orientation = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_blocking)
                angle_z = round(math.degrees(orientation[2]), 2)
                print(f"ball {i}")
                _, ball_pos = sim.simxGetObjectPosition(clientID, ball, -1, sim.simx_opmode_blocking)

                error_x = round(ball_pos[0] - position[0], 2)
                error_y = round(ball_pos[1] - position[1], 2)
                print(f"\nErro X: {error_x}, Erro Y: {error_y}, Angulo Z: {angle_z}")

                if abs(error_x) <= 0.3 and abs(error_y) <= 0.3:
                    sim.simxSetJointTargetVelocity(clientID, motorE, 0, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(clientID, motorD, 0, sim.simx_opmode_blocking)
                    print(f"\nChegou a bola {i}!")
                    i += 1
                    break

                if abs(error_x) > abs(error_y):
                    eixo = 'x'
                else:
                    eixo = 'y'

                print(f"\nEixo escolhido: {eixo}")

                primitiva = self._decidir_movimento(angle_z, eixo, error_x, error_y)
                print(f"\nPrimitiva escolhida: {primitiva}")

                terminou = self.Robot_Pioneer_Primitivas(primitiva, clientID, robot, motorE, motorD, ball)

                if terminou:
                    continue
        
        if len(self.y_out) != len(self.x_out):
            raise ValueError("self.y_out and self.x_out must be of the same length")

        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['x_out', 'y_out'])
            for x, y in zip(self.x_out, self.y_out):
                writer.writerow([x, y])

        print(f"Data saved to {filename}")


        sim.simxFinish(clientID)

if __name__ == "__main__":
    Rvss = 1.6
    RPioneer = 95
    FS = RPioneer / Rvss
    crb01 = Pioneer()
    filename = "Pioneer_primitivas.csv"
    ### DE  Experiment best 0 0.4172###
    kpi_DE = [0.3629, 0.3609, 0.8000, 0.3746, 0.3432]
    kii_DE = [0.1891, 0.3841, 0.0479, 0.0001, 0.0001]
    kdi_DE = [0.0001, 0.0039, 0.0001, 0.0001, 0.0001]
    ### MFO  Experiment best 3 0.3736###
    kpi_MFO = [0.3902, 0.3504, 0.3201, 0.3278, 0.3413]
    kii_MFO = [0.3468, 0.2910, 0.0001, 0.0774, 0.1230]
    kdi_MFO = [0.0001, 0.0050, 0.0001, 0.0002, 0.0049]
    x = [kpi_MFO[4], kii_MFO[4], kdi_MFO[4]]
    deltaT = 0.05
    # crb01.Robot_Pioneer(x, deltaT, filename)
    crb01.executar_primitivas(filename)