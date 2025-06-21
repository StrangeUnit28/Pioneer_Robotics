    # def PID_Controller_phi(self, kp, ki, kd, deltaT, error, interror, fant, Integral_part):

    #     """
    #     Function used to calculate the omega value (output PID) used on the speed function
    #         argument :
    #             kp              (Float) = Proportional constant used on the PID controller
    #             ki              (Float) = Integral constant used on the PID controller
    #             kd              (Float) = Derivative constant used on the PID controller
    #             deltaT          (Float) = Sample time
    #             error           (Float) = Phi error between the robot and the goal point
    #             interror        (Float) = Error Integral
    #             fant            (Float  = Used on the derivative part
    #             Integral_part   (Float) = Integral part

    #         outputs : 
    #            PID              (Float) = Omega value used on the speed function
    #            f                (Float) = f value use on the derivative part
    #            interror         (Float) = Error Integral
    #            Integral_part    (Float) = Integral part
    #     """

    #     ### Max value of saturation in the integral part ###

    #     Integral_saturation = 10

    #     ### Find the filter e ####

    #     raizes = np.roots([kd, kp, ki])
    #     absoluto = abs(raizes)
    #     mayor = max(absoluto)
    #     # print(f'mayor == {mayor}')
    #     Filter_e = 1 / (mayor * 10)

    #     ### Calculate the derivative part ###

    #     unomenosalfaana = mat.exp(-(deltaT / Filter_e))
    #     alfaana = 1 - unomenosalfaana
    #     interror = interror + error
    #     f = unomenosalfaana * fant + alfaana * error
    #     if fant == 0:
    #         deerror = (f / deltaT)
    #     else:
    #         deerror = (float((f - fant) / (deltaT)))

    #     ### Calculate the integral part ###

    #     if Integral_part > Integral_saturation:
    #         Integral_part = Integral_saturation
    #     elif Integral_part < -Integral_saturation:
    #         Integral_part = -Integral_saturation
    #     else:
    #         Integral_part = ki * interror * deltaT

    #     ### Calculate the omega value (PID output) ###

    #     PID = kp * error + Integral_part + deerror * kd

    #     ### Return the principal values ###
    #     # print(f'Integral_part == {Integral_part}')
    #     return PID, f, interror, Integral_part

    # def Robot_Pioneer(self, x, deltaT, filename):

    #     """""
    #     Principal function to simulate the follower ball robot
    #         argument :
    #             kpi              (Float) = Proportional constant used on the PID controller
    #             kii              (Float) = Integral constant used on the PID controller
    #             kdi              (Float) = Derivative constant used on the PID controller
    #             deltaT          (Float) = Sample time

    #         outputs : 
    #            none
    #     """""
    #     kpi = x[0]
    #     kii = x[1]
    #     kdi = x[2]

    #     cont_ar = 0
    #     ### Get the objects within coppeliaSim using the connect_CRB function ###

    #     (clientID, robot, motorE, motorD, ball) = self.connect_Pioneer(19999)

    #     ### Criterio to simulation ###
    #     # Constantes del PID
    #     raizes = np.roots([kdi, kpi, kii])
    #     acumulate_error = 0
    #     absoluto = abs(raizes)
    #     mayor = max(absoluto)
    #     Filter_e = 1 / (mayor * 10)
    #     unomenosalfaana = mat.exp(-(deltaT / Filter_e))
    #     alfaana = 1 - unomenosalfaana
    #     omega_ant = 0
    #     a = 1
    #     angulo_anterior_phid = 0
    #     angulo_anterior_phi_robot = 0

    #     ### Init the principal values ###

    #     Number_Iterations = 0
    #     Time_Sample = []
    #     interror_phi = 0
    #     fant_phi = 0
    #     Integral_part_phi = 0

    #     ### Make the communication with coppeliaSim ###

    #     if (sim.simxGetConnectionId(clientID) != -1):

    #         ### Criterio to simulation ###

    #         ballPos = [-1.75, -0.75]

    #         while (a == 1):

    #             ### important to get valid values and init the phi value ###

    #             if Number_Iterations <= 1:

    #                 s, positiona = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
    #                 # s, ballPos = sim.simxGetObjectPosition(clientID, ball, -1, sim.simx_opmode_streaming)
    #                 s, angle_robot = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_blocking)
    #                 self.phi = 0
    #                 sim.simxSetJointTargetVelocity(clientID, motorE, 0, sim.simx_opmode_blocking)
    #                 sim.simxSetJointTargetVelocity(clientID, motorD, 0, sim.simx_opmode_blocking)
    #             else:

    #                 s, positiona = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
    #                 # s, ballPos = sim.simxGetObjectPosition(clientID, ball, -1, sim.simx_opmode_streaming)
    #                 returnCode, orientation = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_blocking)
    #                 signed = 1
    #                 phi_robot = orientation[2]
    #                 self.phi = phi_robot

    #                 ### Calculate the distance error, the robot stop when arrive the ball ###


    #                 error_distance = math.sqrt((ballPos[1] - positiona[1]) ** 2 + (ballPos[0] - positiona[0]) ** 2)

    #                 # print(f'Angle robot ==> {angle_robot}')

    #                 if error_distance >= self.Min_error_distance:
    #                     ### Calculate the phid (see georgia tech course) ###

    #                     phid = math.atan2(ballPos[1] - positiona[1], ballPos[0] - positiona[0])

    #                     #### Proportional controller of liner velocity ####

    #                     # controller_Linear = self.v_linear * error_distance
    #                     controller_Linear = self.v_linear
    #                     lock_stop_simulation = 0

    #                 else:
    #                     cont_ar = cont_ar + 1
    #                     if cont_ar == 1:
    #                         ballPos = [-0.7, -1.375]
    #                     elif cont_ar == 2:
    #                         ballPos = [-0.175, -0.75]
    #                     elif cont_ar == 3:
    #                         ballPos = [-0.2, -2.05]
    #                     elif cont_ar == 4:
    #                         ballPos = [-1.74, -2.075]
    #                     elif cont_ar == 5:
    #                         phid = 1.57  # 90
    #                         controller_Linear = 0
    #                     elif cont_ar == 6:

    #                         lock_stop_simulation = 1


    #                 ### Phi error to send the PID controller

    #                 # phid = phid + 1.5708  # sum 90

    #                 # Calcula la diferencia entre el ángulo actual y el anterior
    #                 diferencia_phid = phid - angulo_anterior_phid
    #                 diferencia_phi = self.phi - angulo_anterior_phi_robot
    #                 # Si la diferencia es mayor que π, ajusta restando 2π
    #                 if diferencia_phid > math.pi:
    #                     phid -= 2 * math.pi
    #                 # Si la diferencia es menor que -π, ajusta sumando 2π
    #                 elif diferencia_phid < -math.pi:
    #                     phid += 2 * math.pi

    #                 # Si la diferencia es mayor que π, ajusta restando 2π
    #                 if diferencia_phi > math.pi:
    #                     self.phi -= 2 * math.pi
    #                 # Si la diferencia es menor que -π, ajusta sumando 2π
    #                 elif diferencia_phi < -math.pi:
    #                     self.phi += 2 * math.pi

    #                 # Actualiza el ángulo anterior
    #                 angulo_anterior_phid = phid
    #                 angulo_anterior_phi_robot = self.phi

    #                 print(f'phid == > {phid}, self.phi ==> {self.phi}, error ==> {phid - self.phi}')
    #                 error_phi = phid - self.phi

    #                 ### Acumulative distance error ###

    #                 acumulate_error = acumulate_error + abs(phid - self.phi)

    #                 ### Implement the PID controller ###

    #                 omega, fant_phi, interror_phi, Integral_part_phi = self.PID_Controller_phi(kpi, kii, kdi, deltaT,
    #                                                                                            error_phi, interror_phi,
    #                                                                                            fant_phi,
    #                                                                                            Integral_part_phi)

    #                 if omega >= 100 or omega <= -100:
    #                     omega = omega_ant
    #                 else:
    #                     omega_ant = omega

    #                 self.posError.append(error_distance)

    #                 ### Calculate the speed right and left based on the topology robot ###

    #                 vl, vd, a = self.Speed_Pioneer(controller_Linear, omega, lock_stop_simulation, signed,
    #                                            abs(phid - self.phi))

    #                 print(f'Speed lef == {vl}, Speed Right == {vd}')
    #                 ### Send the speed values to coppeliasim simulato ###

    #                 sim.simxSetJointTargetVelocity(clientID, motorE, vl, sim.simx_opmode_blocking)
    #                 sim.simxSetJointTargetVelocity(clientID, motorD, vd, sim.simx_opmode_blocking)

    #                 ### update the time simulation and the simulation iteration

    #             Number_Iterations = Number_Iterations + 1
    #             Time_Sample.append(Number_Iterations * deltaT)
    #             if Number_Iterations >= 60:
    #                 a == 0
    #             # time.sleep(0.5)
    #             ### Save the robot position ###ç
    #             # Detener la simulación
    #             self.y_out.append(positiona[1])
    #             self.x_out.append(positiona[0])
    #         if len(self.y_out) != len(self.x_out):
    #             raise ValueError("self.y_out and self.x_out must be of the same length")

    #             # Open the CSV file for writing
    #         with open(filename, mode='w', newline='') as file:
    #             writer = csv.writer(file)

    #             # Write the header
    #             writer.writerow(['x_out', 'y_out'])

    #             # Write the data rows
    #             for x, y in zip(self.x_out, self.y_out):
    #                 writer.writerow([x, y])

    #         print(f"Data saved to {filename}")

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

    # def Robot_Pioneer_Primitivas(self, primitiva, clientID, robot, motorE, motorD, ball):

    #     a = 1
    #     Number_Iterations = 0

    #     if (sim.simxGetConnectionId(clientID) != -1):

    #         ### Criterio to simulation ###

    #         while (a == 1):

    #             ### important to get valid values and init the phi value ###

    #             if Number_Iterations <= 1:

    #                 s, positiona = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
    #                 s, angle_robot = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_blocking)
    #                 s, ballPos = sim.simxGetObjectPosition(clientID, ballHandler[i], -1, sim.simx_opmode_streaming)
    #                 self.phi = 0
    #                 sim.simxSetJointTargetVelocity(clientID, motorE, 0, sim.simx_opmode_blocking)
    #                 sim.simxSetJointTargetVelocity(clientID, motorD, 0, sim.simx_opmode_blocking)
    #                 start_position = positiona[:2]
    #                 start_z_degrees = math.degrees(angle_robot[2])
    #             else:

    #                 s, positiona = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
    #                 returnCode, orientation = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_blocking)
    #                 s, ballPos = sim.simxGetObjectPosition(clientID, ballHandler[i], -1, sim.simx_opmode_streaming)
                    
    #                 new_position = positiona[:2]
    #                 new_z_degrees = round(math.degrees(orientation[2]), 2)
    #                 print(f'start_position = {start_position}, new_position = {new_position}, start_y_degrees = {start_z_degrees} new_y_degrees = {new_z_degrees}')
                    
    #                 # controller_Linear = self.v_linear * error_distance
    #                 controller_Linear = self.v_linear
    #                 lock_stop_simulation = 0
                    
    #                 # Angle and Position variation
    #                 d_phi = abs(new_z_degrees) - abs(start_z_degrees)
    #                 dx = round(new_position[0] - start_position[0], 2)
    #                 dy = round(new_position[1] - start_position[1], 2)
    #                 print(f'\nd_phi = {d_phi}, dx = {dx}, dy = {dy}')
                    
    #                 # Distance to ball
    #                 error_distance_x = ballPos[0] - positiona[0]
    #                 error_distance_y = ballPos[1] - positiona[1] 
    #                 print(f'error_distance_x = {error_distance_x}, error_distance_y = {error_distance_y}')
                    
    #                 if primitiva == "forward": # -0.5
    #                     if dx >= 0.49 or dy >= 0.49:
    #                         # vl = 0
    #                         # vd = 0
    #                         # a = 0
    #                         vl, vd, a = map(lambda x: 0, range(3))
    #                     else:
    #                         omega = 0
    #                         vl, vd, a = self.Speed_Pioneer(controller_Linear, omega, lock_stop_simulation, 1, 0)
    #                 if primitiva == "right": # -0.5 -0.5
    #                     if d_phi <= -89.0:
    #                         # vl = 0
    #                         # vd = 0
    #                         # a = 0
    #                         vl, vd, a = map(lambda x: 0, range(3))
    #                     else:
    #                         radio_ideal = -0.5
    #                         omega = (self.v_linear/radio_ideal)
    #                         vl, vd, a = self.Speed_Pioneer(controller_Linear, omega, lock_stop_simulation, 1, 0)
    #                 if primitiva == "left": # -0.5 -0.5
    #                     if d_phi >= 89.0:
    #                         # vl = 0
    #                         # vd = 0
    #                         # a = 0
    #                         vl, vd, a = map(lambda x: 0, range(3))
    #                     else:
    #                         radio_ideal = 0.5
    #                         omega = (self.v_linear / radio_ideal)
    #                         vl, vd, a = self.Speed_Pioneer(controller_Linear, omega, lock_stop_simulation, 1, 0)
                    
    #                 print(f'Speed lef == {vl}, Speed Right == {vd}')

    #                 sim.simxSetJointTargetVelocity(clientID, motorE, vl, sim.simx_opmode_blocking)
    #                 sim.simxSetJointTargetVelocity(clientID, motorD, vd, sim.simx_opmode_blocking)               

    #             Number_Iterations += 1

    # def executar_primitivas(self):
    #     (clientID, robot, motorE, motorD, ballHandler) = self.connect_Pioneer(19999)
        
    #     i = 0

    #     if sim.simxGetConnectionId(clientID) == -1:
    #         print("Falha ao conectar com o CoppeliaSim.")
    #         return
        

    #     self.Robot_Pioneer_Primitivas("forward", clientID, robot, motorE, motorD, ballHandler[i])


    #     if 1.5 > abs(new_z_degrees) >= 0:

    #     elif 88.5 > abs(new_z_degrees) < 91.5:

    #     elif 178.5 > abs(new_z_degrees) < 181.5:

    #     elif 268.5 > abs(new_z_degrees) < 271.5:


    #     sim.simxFinish(clientID)

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
    filename = "Pioneer_experiment.csv"
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