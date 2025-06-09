import math
import sim
import numpy as np
import math as mat
import time
import csv

class Pioneer():
    def __init__(self):
        self.y_out = []
        self.x_out = []
        self.phi = 0
        self.v_max_wheels = 15
        self.v_min_wheels = -15
        self.v_linear = 15
        self.posError = []
        self.Min_error_distance = 0.5

    def connect_Pioneer(self, port):

        sim.simxFinish(-1)
        clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)
        if clientID == 0:
            print("Connect to", port)
        else:
            print("Can not connect to", port)


        returnCode, robot = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx', sim.simx_opmode_blocking)  
        returnCode, MotorE = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)  
        returnCode, MotorD = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)  
        
        ballHandler = [0, 0, 0 , 0]
        returnCode, ballHandler[0] = sim.simxGetObjectHandle(clientID, 'ball_1', sim.simx_opmode_blocking)
        returnCode, ballHandler[1] = sim.simxGetObjectHandle(clientID, 'ball_2', sim.simx_opmode_blocking)
        returnCode, ballHandler[2] = sim.simxGetObjectHandle(clientID, 'ball_3', sim.simx_opmode_blocking)
        returnCode, ballHandler[3] = sim.simxGetObjectHandle(clientID, 'ball_4', sim.simx_opmode_blocking)

        return clientID, robot, MotorE, MotorD, ballHandler
    
    def Speed_Pioneer(self, U, omega, lock_stop_simulation, signed, error_phi):

        L = 381  # Distance between the wheels
        R = 95  # Wheel radio

        vd = ((2 * (U) + omega * L) / (2 * R))
        vl = ((2 * (U) - omega * L) / (2 * R))
        a = 1
        
        ### the first time #####
        Max_Speed = self.v_max_wheels
        Min_Speed = self.v_min_wheels

        ### Saturation upper speed ###
        if vd >= Max_Speed:
            vd = Max_Speed
        if vd <= Min_Speed:
            vd = Min_Speed

        ### Saturation lower speed ###
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
    
    def PID_Controller_phi(self, kp, ki, kd, deltaT, error, interror, fant, Integral_part):

        ### Max value of saturation in the integral part ###
        Integral_saturation = 10

        ### Find the filter e ####
        raizes = np.roots([kd, kp, ki])
        absoluto = abs(raizes)
        mayor = max(absoluto)
        Filter_e = 1 / (mayor * 10)

        ### Calculate the derivative part ###
        unomenosalfaana = mat.exp(-(deltaT / Filter_e))
        alfaana = 1 - unomenosalfaana
        interror = interror + error
        f = unomenosalfaana * fant + alfaana * error
        if fant == 0:
            deerror = (f / deltaT)
        else:
            deerror = (float((f - fant) / (deltaT)))

        ### Calculate the integral part ###
        if Integral_part > Integral_saturation:
            Integral_part = Integral_saturation
        elif Integral_part < -Integral_saturation:
            Integral_part = -Integral_saturation
        else:
            Integral_part = ki * interror * deltaT

        ### Calculate the omega value (PID output) ###
        PID = kp * error + Integral_part + deerror * kd

        ### Return the principal values ###
        return PID, f, interror, Integral_part
    
    def moveFoward(self):

    def turnLeft(self):

    def turnRight(self): 

    def Robot_Pioneer(self, x, deltaT, filename):

        kpi = x[0]
        kii = x[1]
        kdi = x[2]
        ### Get the objects within coppeliaSim using the connect_CRB function ###

        (clientID, robot, motorE, motorD, ballHandler) = self.connect_Pioneer(19999)
        ### Criterio to simulation ###
        # Constantes del PID
        raizes = np.roots([kdi, kpi, kii])
        acumulate_error = 0
        absoluto = abs(raizes)
        mayor = max(absoluto)
        Filter_e = 1 / (mayor * 10)
        unomenosalfaana = mat.exp(-(deltaT / Filter_e))
        alfaana = 1 - unomenosalfaana
        omega_ant = 0
        a = 1
        angulo_anterior_phid = 0
        angulo_anterior_phi_robot = 0

        ### Init the principal values ###

        Number_Iterations = 0
        Time_Sample = []
        interror_phi = 0
        fant_phi = 0
        Integral_part_phi = 0

        ### Make the communication with coppeliaSim ###

        if (sim.simxGetConnectionId(clientID) != -1):

            ### Criterio to simulation ###

            i = 0 # Ball's sequence in the array ballHandler

            while (i < len(ballHandler)):

                ### important to get valid values and init the phi value ###

                if Number_Iterations <= 1:

                    s, positiona = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
                    s, ballPos = sim.simxGetObjectPosition(clientID, ballHandler[i], -1, sim.simx_opmode_streaming)
                    s, angle_robot = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_blocking)

                    self.phi = 0
                    sim.simxSetJointTargetVelocity(clientID, motorE, 0, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(clientID, motorD, 0, sim.simx_opmode_blocking)
                else:
                    s, positiona = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
                    s, ballPos = sim.simxGetObjectPosition(clientID, ballHandler[i], -1, sim.simx_opmode_streaming)
                    returnCode, orientation = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_blocking)

                    signed = 1
                    phi_robot = orientation[2]
                    self.phi = phi_robot

                    ### Calculate the distance error, the robot stop when arrive the ball ###

                    error_distance = math.sqrt((ballPos[1] - positiona[1]) ** 2 + (ballPos[0] - positiona[0]) ** 2)

                    # print(f'Angle robot ==> {angle_robot}')

                    if error_distance >= self.Min_error_distance:
                        ### Calculate the phid (see georgia tech course) ###

                        phid = math.atan2(ballPos[1] - positiona[1], ballPos[0] - positiona[0])

                        #### Proportional controller of liner velocity ####

                        controller_Linear = self.v_linear * error_distance
                        # controller_Linear = self.v_linear
                        lock_stop_simulation = 0

                    else:

                        print(f"\nAlcançou a bola {i + 1} com erro = {error_distance}")
                        i += 1

                        phid = 1.57  # 90
                        controller_Linear = 0


                    ### Phi error to send the PID controller

                    # phid = phid + 1.5708  # sum 90

                    # Calcula la diferencia entre el ángulo actual y el anterior
                    diferencia_phid = phid - angulo_anterior_phid
                    diferencia_phi = self.phi - angulo_anterior_phi_robot
                    # Si la diferencia es mayor que π, ajusta restando 2π
                    if diferencia_phid > math.pi:
                        phid -= 2 * math.pi
                    # Si la diferencia es menor que -π, ajusta sumando 2π
                    elif diferencia_phid < -math.pi:
                        phid += 2 * math.pi

                    # Si la diferencia es mayor que π, ajusta restando 2π
                    if diferencia_phi > math.pi:
                        self.phi -= 2 * math.pi
                    # Si la diferencia es menor que -π, ajusta sumando 2π
                    elif diferencia_phi < -math.pi:
                        self.phi += 2 * math.pi

                    # Actualiza el ángulo anterior
                    angulo_anterior_phid = phid
                    angulo_anterior_phi_robot = self.phi

                    print(f'phid == > {phid}, self.phi ==> {self.phi}, error ==> {phid - self.phi}')
                    error_phi = phid - self.phi

                    ### Acumulative distance error ###

                    acumulate_error = acumulate_error + abs(phid - self.phi)

                    ### Implement the PID controller ###

                    omega, fant_phi, interror_phi, Integral_part_phi = self.PID_Controller_phi(kpi, kii, kdi, deltaT,
                                                                                               error_phi, interror_phi,
                                                                                               fant_phi,
                                                                                               Integral_part_phi)

                    if omega >= 100 or omega <= -100:
                        omega = omega_ant
                    else:
                        omega_ant = omega

                    self.posError.append(error_distance)

                    ### Calculate the speed right and left based on the topology robot ###

                    vl, vd, a = self.Speed_Pioneer(controller_Linear, omega, lock_stop_simulation, signed,
                                               abs(phid - self.phi))

                    print(f'Speed lef == {vl}, Speed Right == {vd}')
                    ### Send the speed values to coppeliasim simulator ###

                    sim.simxSetJointTargetVelocity(clientID, motorE, vl, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(clientID, motorD, vd, sim.simx_opmode_blocking)

                    ### update the time simulation and the simulation iteration

                Number_Iterations = Number_Iterations + 1
                Time_Sample.append(Number_Iterations * deltaT)
                # if Number_Iterations >= 10000:
                #     a == 0
                # time.sleep(0.5)
                ### Save the robot position ###ç
                # Detener la simulación
                self.y_out.append(positiona[1])
                self.x_out.append(positiona[0])

            print("\nTodos os objetivos foram alcançados.")

            # Stop motors
            sim.simxSetJointTargetVelocity(clientID, motorE, 0, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(clientID, motorD, 0, sim.simx_opmode_blocking)

            # End simulation
            sim.simxFinish(clientID)

            if len(self.y_out) != len(self.x_out):
                raise ValueError("self.y_out and self.x_out must be of the same length")

                # Open the CSV file for writing
            with open(filename, mode='w', newline='') as file:
                writer = csv.writer(file)

                # Write the header
                writer.writerow(['x_out', 'y_out'])

                # Write the data rows
                for x, y in zip(self.x_out, self.y_out):
                    writer.writerow([x, y])

            print(f"Data saved to {filename}")


if __name__ == "__main__":
    Rvss = 1.6
    RPioneer = 95
    # FS = RPioneer / Rvss
    robot = Pioneer()
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
    robot.Robot_Pioneer(x, deltaT, filename)
    