import math
import numpy as np
import random

radianToDegree = 180/math.pi
degreeToRadian = math.pi/180


class Quadrotor(object):
    def __init__(self, name, specs, initialState, initialInput, attitudeControllerPID, positionControllerPID):
        self.name = name
        self.t = 0 # s
        self.dt = 0.01 # s

        self.mass = specs["mass"] # kg
        self.inertia = specs["inertia"]  # [Ixx, Iyy, Izz] # kg m^2
        self.armLength = specs["armLength"] #, m

        # Quadrotor Initial State
        self.position = np.array(initialState[0], dtype=float) # m
        self.position_dot = np.array(initialState[1], dtype=float) # m/s
        self.angles = np.array(initialState[2], dtype=float)  # [phi theta psi] # rad
        self.angles_dot = np.array(initialState[3], dtype=float) # rad / s
        self.state_dot = np.array([[0, 0, 0],  # position_dot
                                   [0, 0, 0],  # position_dot_dot
                                   [0, 0, 0],  # angles_dot
                                   [0, 0, 0]], dtype=float)  # angles_dot_dot

        # Initiate Controlled Variable
        self.thrust = initialInput[0] # kg m rad^2/s^2
        self.moments = [initialInput[1], initialInput[2], initialInput[3]] # kg m^2 rad^2/s^2

        self.thrust_max = 60
        self.thrust_min = 0
        self.moment_max = 100
        self.moment_min = -100

        # Attitude Controller with PID
        self.KP_phi = attitudeControllerPID[0][0]
        self.KI_phi = attitudeControllerPID[0][1]
        self.KD_phi = attitudeControllerPID[0][2]

        self.KP_theta = attitudeControllerPID[1][0]
        self.KI_theta = attitudeControllerPID[1][1]
        self.KD_theta = attitudeControllerPID[1][2]

        self.KP_psi = attitudeControllerPID[2][0]
        self.KI_psi = attitudeControllerPID[2][1]
        self.KD_psi = attitudeControllerPID[2][2]

        self.phi_err = 0
        self.phi_err_prev = 0
        self.phi_err_sum = 0

        self.theta_err = 0
        self.theta_err_prev = 0
        self.theta_err_sum = 0

        self.psi_err = 0
        self.psi_err_prev = 0
        self.psi_err_sum = 0

        self.KP_zdot = attitudeControllerPID[3][0]
        self.KI_zdot = attitudeControllerPID[3][1]
        self.KD_zdot = attitudeControllerPID[3][2]

        self.z_dot_err = 0
        self.z_dot_err_prev = 0
        self.z_dot_err_sum = 0

        # Position Controller with PID
        self.x_des_dot_prev = 0
        self.y_des_dot_prev = 0
        self.z_des_dot_prev = 0
        self.x_dot_prev = 0
        self.y_dot_prev = 0
        self.z_dot_prev = 0

        self.KP_x = positionControllerPID[0][0]
        self.KI_x = positionControllerPID[0][1]
        self.KD_x = positionControllerPID[0][2]

        self.KP_y = positionControllerPID[1][0]
        self.KI_y = positionControllerPID[1][1]
        self.KD_y = positionControllerPID[1][2]

        self.KP_z = positionControllerPID[2][0]
        self.KI_z = positionControllerPID[2][1]
        self.KD_z = positionControllerPID[2][2]

        self.x_err = 0
        self.x_err_prev = 0
        self.x_err_sum = 0

        self.y_err = 0
        self.y_err_prev = 0
        self.y_err_sum = 0

        self.z_err = 0
        self.z_err_prev = 0
        self.z_err_sum = 0

        self.x_dot_err = 0
        self.x_dot_err_sum = 0

        self.y_dot_err = 0
        self.y_dot_err_sum = 0

        self.phi_pos_err = 0
        self.phi_pos_err_sum = 0

        self.theta_pos_err = 0
        self.theta_pos_err_sum = 0

        self.z_pos_err = 0
        self.z_pos_err_sum = 0

    def calculateFrictionForce(self, velocity):
        if velocity>0 :
            print('friction Force', -5*(velocity**2))
            return -10*(velocity**2)
        else:
            print('friction Force', 5*(velocity**2))
            return 10*(velocity**2)

    def generateNoise(self):
        self.noise = [random.gauss(0, 5), random.gauss(
            0, 5), random.gauss(0, 5)]

    def getState(self):
        return [self.position, self.position_dot, self.angles*radianToDegree, self.angles_dot*radianToDegree]

    def getBodyPosition(self):

        # Quadrotor Body (Rotor), format: ([x,y,z])
        # direction : +x
        rotor1 = [self.position[0] + math.cos(self.angles[1])*self.armLength + math.cos(self.angles[2])*self.armLength, self.position[1] - math.sin(self.angles[2])*self.armLength,
                  self.position[2] + math.sin(self.angles[1])*self.armLength]

        # direction : -x
        rotor2 = [self.position[0] - math.cos(self.angles[1])*self.armLength - math.cos(self.angles[2])*self.armLength, self.position[1] + math.sin(self.angles[2])*self.armLength,
                  self.position[2] - math.sin(self.angles[1])*self.armLength]
        
        # direction : +y
        rotor3 = [self.position[0] + math.sin(self.angles[2])*self.armLength, self.position[1] + math.cos(
            self.angles[0]) * self.armLength + math.cos(self.angles[2])*self.armLength, self.position[2] - math.sin(self.angles[0])*self.armLength]

        # direction : -y
        rotor4 = [self.position[0] - math.sin(self.angles[2])*self.armLength, self.position[1] - math.cos(
            self.angles[0]) * self.armLength - math.cos(self.angles[2])*self.armLength, self.position[2] + math.sin(self.angles[0])*self.armLength]

        return np.array([[rotor1[0], rotor2[0], rotor3[0], rotor4[0]], [rotor1[1], rotor2[1], rotor3[1], rotor4[1]], [rotor1[2], rotor2[2], rotor3[2], rotor4[2]]])

    def updateState(self):
        self.t = self.t + self.dt

        # Evaluate Equation of Motions
        phi = self.angles[0]
        theta = self.angles[1]
        psi = self.angles[2]      

        if self.thrust > self.thrust_max:
            self.thrust = self.thrust_max
        if self.thrust < self.thrust_min:
            self.thrust = self.thrust_min

        for moment in self.moments:
            if moment > self.moment_max:
                moment = self.moment_max
            if moment < self.moment_min:
                moment = self.moment_min

        # Translational Motion
        self.state_dot[0] = self.position_dot
        self.state_dot[1][0] = -1/self.mass*((math.sin(psi)*math.sin(
            phi)+math.cos(psi)*math.sin(theta)*math.cos(phi))*self.thrust - self.calculateFrictionForce(self.position_dot[0]))
        self.state_dot[1][1] = -1/self.mass*((-math.cos(psi)*math.sin(
            phi)+math.sin(psi)*math.sin(theta)*math.cos(phi))*self.thrust - self.calculateFrictionForce(self.position_dot[1]))
        self.state_dot[1][2] = 1/self.mass * \
            (math.cos(theta)*math.cos(phi))*self.thrust - 9.81 + self.calculateFrictionForce(self.position_dot[2])/self.mass

        print('acceleration', self.state_dot[1])

        # Rotational Motion
        p = self.angles_dot[0]
        q = self.angles_dot[1]
        r = self.angles_dot[2]
        self.state_dot[2][0] = p + \
            math.sin(psi)*math.tan(theta)*q+math.cos(phi)*math.tan(theta)*r
        self.state_dot[2][1] = math.cos(phi)*q-math.sin(phi)*r
        self.state_dot[2][2] = math.sin(
            phi)/math.cos(theta)*q+math.cos(phi)/math.cos(theta)*r

        Ixx = self.inertia[0]
        Iyy = self.inertia[1]
        Izz = self.inertia[2]
        self.state_dot[3][0] = (Iyy-Izz)/Ixx*q*r+self.moments[0]/Ixx
        self.state_dot[3][1] = (Izz-Ixx)/Iyy*r*p+self.moments[1]/Iyy
        self.state_dot[3][2] = (Ixx-Iyy)/Izz*p*q+self.moments[2]/Izz

        # Update the state
        self.position = self.position + self.state_dot[0] * self.dt
        self.position_dot = self.position_dot + self.state_dot[1]*self.dt
        self.angles = self.angles + self.state_dot[2] * self.dt
        self.angles_dot = self.angles_dot + self.state_dot[3] * self.dt

    def controlAttitude(self, attitudeTarget):
        # Add sensor noise (with white noise) with random.gauss(mu, sigma)
        self.generateNoise()
        # + self.noise[0]
        self.phi_err = attitudeTarget[0]*degreeToRadian - self.angles[0]
        # + self.noise[1]
        self.theta_err = attitudeTarget[1]*degreeToRadian - self.angles[1]
        # + self.noise[2]
        self.psi_err = attitudeTarget[2]*degreeToRadian - self.angles[2]

        # Calculate output for controlled variable
        '''
        self.moments[0] = phi
        self.moments[1] = theta
        self.moments[2] = psi
        self.thrust = position -> z_dot
        '''
        self.moments[0] = self.KP_phi * self.phi_err + self.KI_phi*self.phi_err_sum + \
            self.KD_phi * (self.phi_err - self.phi_err_prev)/self.dt

        self.moments[1] = self.KP_theta * self.theta_err + self.KI_theta*self.theta_err_sum + \
            self.KD_theta * (self.theta_err - self.theta_err_prev)/self.dt

        self.moments[2] = self.KP_psi * self.psi_err + self.KI_psi*self.psi_err_sum + \
            self.KD_psi * (self.psi_err - self.psi_err_prev)/self.dt

        self.phi_err_prev = self.phi_err
        self.phi_err_sum = self.phi_err_sum + self.phi_err
        self.theta_err_prev = self.theta_err
        self.theta_err_sum = self.theta_err_sum + self.theta_err
        self.psi_err_prev = self.psi_err
        self.psi_err_sum = self.psi_err_sum + self.psi_err

        self.z_dot_err = attitudeTarget[3] - self.position_dot[2]

        self.thrust = (self.KP_zdot * self.z_dot_err + self.KI_zdot *
                                          self.z_dot_err_sum + self.KD_zdot*(self.z_dot_err - self.z_dot_err_prev)/self.dt)

        self.z_dot_err_prev = self.z_dot_err
        self.z_dot_err_sum = self.z_dot_err_sum + self.z_dot_err

    def controlPosition(self, positionTarget):
        # Keknya bener
        # self.x_err = positionTarget[0] - self.position[0]
        # self.y_err = positionTarget[1] - self.position[1]
        # self.z_err = positionTarget[2] - self.position[2]

        # attitudeTarget = [0,0,0,0] # psi theta phi zdot
        # attitudeTarget[0] = self.KP_y * self.y_err
        # + self.KI_y * self.y_err_sum
        # + self.KD_y * (self.y_err - self.y_err_prev) / self.dt

        # self.y_err_prev = self.y_err
        # self.y_err_sum = self.y_err_sum + self.y_err

        # attitudeTarget[1] = self.KP_x * self.x_err
        # + self.KI_x * self.x_err_sum
        # + self.KD_x * (self.x_err - self.x_err_prev) / self.dt

        # self.x_err_prev = self.x_err
        # self.x_err_sum = self.x_err_sum + self.x_err

        # attitudeTarget[3] = self.KP_z * self.z_err
        # + self.KI_z * self.z_err_sum
        # + self.KD_z * (self.z_err - self.z_err_prev) / self.dt

        # self.z_err_prev = self.z_err
        # self.z_err_sum = self.z_err_sum + self.z_err

        # print('Degree : ', self.getState()[2])
        # print('Attitude Target', attitudeTarget)

        # self.controlAttitude(attitudeTarget)

        # Control y pos
        self.y_err = positionTarget[1] - self.position[1] # / m

        y_des_dot = (self.y_err)/self.dt # m/s
        self.y_dot_err = y_des_dot - self.position_dot[1] # m/s

        self.phi_pos_err = self.KP_y*self.y_err + self.KI_y * self.y_err_sum + \
            self.KD_y*(y_des_dot - self.position_dot[1]) - self.angles[0]
        self.moments[0] = self.KP_phi * (self.phi_pos_err)
        + self.KI_phi*(self.phi_pos_err_sum) + \
            self.KD_phi * (self.KP_y*self.y_dot_err + self.KI_y*self.y_dot_err_sum+self.KD_phi * ((y_des_dot -
                                                                                                   self.y_des_dot_prev)/self.dt - (self.position_dot[1] - self.y_dot_prev)/self.dt) - self.angles_dot[1])
        self.y_dot_prev = self.position_dot[1]
        self.y_des_dot_prev = y_des_dot

        self.y_err_prev = self.y_err
        self.y_err_sum = self.y_err_sum + self.y_err
        self.y_dot_err_sum = self.y_dot_err_sum + self.y_dot_err

        self.phi_pos_err_sum = self.phi_pos_err_sum + self.phi_pos_err

        # Control x pos
        self.x_err = positionTarget[0] - self.position[0]
        x_des_dot = (positionTarget[0] - self.position[0])/self.dt
        self.x_dot_err = x_des_dot - self.position_dot[0]
        
        self.theta_pos_err = self.KP_x*self.x_err + self.KI_x * self.x_err_sum + \
            self.KD_x*(x_des_dot - self.position_dot[0]) - self.angles[1]
        self.moments[1] = self.KP_theta * (self.theta_pos_err)
        + self.KI_theta*(self.theta_pos_err_sum) + \
            self.KD_theta * (self.KP_x*self.x_dot_err + self.KI_x*self.x_dot_err_sum+self.KD_theta * (
                (x_des_dot-self.x_des_dot_prev)/self.dt - (self.position_dot[0] - self.x_dot_prev)/self.dt) - self.angles_dot[0])

        self.x_dot_prev = self.position_dot[0]
        self.x_des_dot_prev = x_des_dot

        self.x_err_prev = self.x_err
        self.x_err_sum = self.x_err_sum + self.x_err
        self.x_dot_err_sum = self.x_dot_err_sum + self.x_dot_err

        self.theta_pos_err_sum = self.theta_pos_err_sum + self.theta_pos_err

        # Control yaw
        self.moments[2] = self.KP_psi * self.psi_err + self.KI_psi*self.psi_err_sum + \
            self.KD_psi * (self.psi_err - self.psi_err_prev)/self.dt

        # Control z      
        self.z_err = positionTarget[2] - self.position[2]
        z_des_dot = (positionTarget[2] - self.position[2])
        self.z_dot_err = z_des_dot - self.position_dot[2]

        self.z_pos_err = self.KP_z*self.z_err + self.KI_z * self.z_err_sum + \
            self.KD_z*(z_des_dot - self.position_dot[2]) - self.position_dot[2]

        self.thrust = self.KP_zdot * self.z_pos_err 
        + self.KI_zdot * self.z_pos_err_sum 
        + self.KD_zdot*(self.KP_z * self.z_dot_err + self.KI_z *self.z_dot_err_sum + self.KD_zdot *(
            (z_des_dot - self.z_des_dot_prev)/self.dt - (self.position_dot[2] - self.z_dot_prev)/self.dt)- (self.position_dot[2] - self.z_dot_prev)/self.dt)
       
        print('z_des_dot', z_des_dot)
        print('z_dot_real', self.position_dot[2])
        print('zdot_err', self.z_dot_err)
        print('thrust before', self.thrust)

        self.z_dot_prev = self.position_dot[2]
        self.z_des_dot_prev = z_des_dot
        self.z_err_prev = self.z_err
        self.z_err_sum = self.z_err_sum + self.z_err
        self.z_dot_err_sum = self.z_dot_err_sum + self.z_dot_err

        self.z_pos_err_sum = self.z_pos_err_sum + self.z_dot_err

    def controlSwarm(self):
        # with APF
        return 'in progress'
