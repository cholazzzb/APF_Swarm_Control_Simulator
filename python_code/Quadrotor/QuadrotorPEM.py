import math
import numpy as np
import random

import sys
sys.path.append('../../')
from Agent import Agent

radianToDegree = 180/math.pi
degreeToRadian = math.pi/180

# Rotation Matrix -> QTGM (Quadrotor To Global Matrix)
'''
phi =
theta =
psi = + -> Counter-clockwise (rotate quad coordinate to global coordinate in counter-clockwise in this degree)
'''
# Rotation Matrix -> QTGM (Quadrotor To Global Matrix) Only from Yaw (Psi)
def QTGMRotationMatrix(psi):
    return np.array([
            [math.cos(psi), -math.sin(psi), 0],
            [math.sin(psi), math.cos(psi), 0],
            [0, 0, 1]])

def QTGM(angles, vector):
    psi = angles[2]
    rotationMatrix = QTGMRotationMatrix(psi)

    return np.round(rotationMatrix.dot(vector), 2)

# Global to Quadrotor Coordinate
def GTQM(angles, vector):
    psi = angles[2]
    rotationMatrix = QTGMRotationMatrix(psi).T

    return np.round(rotationMatrix.dot(vector), 2)

class QuadrotorPEM(object):
    def __init__(self, index, name, specs, initialState, initialInput, positionControllerPID):
        self.index = index
        self.name = name
        self.t = 0  # s
        self.dt = 0.7 # s

        self.mass = specs["mass"]  # kg
        self.inertia = specs["inertia"]  # [Ixx, Iyy, Izz] # kg m^2
        self.armLength = specs["armLength"]  # , m

        # Quadrotor Initial State
        self.position = np.array(initialState[0], dtype=float)  # m
        self.position_dot = np.array(initialState[1], dtype=float)  # m/s
        # [phi theta psi] # rad
        self.angles = np.array(initialState[2], dtype=float)
        self.angles_dot = np.array(initialState[3], dtype=float)  # rad / s
        self.state_dot = np.array([[0, 0, 0],  # position_dot
                                   [0, 0, 0],  # position_dot_dot
                                   [0, 0, 0],  # angles_dot
                                   [0, 0, 0]], dtype=float)  # angles_dot_dot
        self.x1 = 0
        self.x2 = 0
        self.x3 = 0
        self.x4 = 0
        self.x5 = initialState[0][0]
        self.x6 = 0
        self.x7 = initialState[0][1]
        self.x8 = initialState[0][2]

        self.x1_dot = 0
        self.x2_dot = 0
        self.x3_dot = 0
        self.x4_dot = 0
        self.x5_dot = 0
        self.x6_dot = 0
        self.x7_dot = 0
        self.x8_dot = 0
        
        # Initiate Controlled Variable
        self.inputT = initialInput[0]  
        self.inputRoll = initialInput[1] 
        self.inputPitch = initialInput[2]
        self.inputYaw = initialInput[3]

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

        self.z_pos_err = 0
        self.z_pos_err_sum = 0

        # Swarm Control
        self.targetPosition = []

        # AR max angles
        # self.max_phi_theta_psi = 30*degreeToRadian #degree
        # For TA
        self.max_phi_theta_psi = 9*degreeToRadian 
        self.max_input = 1
        self.min_input = -1

        # Yaw Control
        self.yaw_target = 0

    def calculateFrictionForce(self, velocity):
        if velocity > 0:
            # print('friction Force', -5*(velocity**2))
            return -10*(velocity**2)
        else:
            # print('friction Force', 5*(velocity**2))
            return 10*(velocity**2)

    def generateNoise(self):
        self.noise = [random.gauss(0, 5), random.gauss(
            0, 5), random.gauss(0, 5)]

    def getState(self):
        return [self.position, self.position_dot, self.angles*radianToDegree, self.angles_dot*radianToDegree]

    def getInput(self):
        return [self.inputT, self.inputPitch, self.inputRoll, self.inputYaw]

    def getBodyPosition(self):

        # Quadrotor Body (Rotor), format: ([x,y,z])
        # Using Rotation Matrix
        # direction : +x
        rotor1 = self.position + QTGM(self.angles, np.array([self.armLength, 0, 0]))
        # direction : -x
        rotor2 = self.position + QTGM(self.angles, np.array([-self.armLength, 0, 0]))
        # direction : +y (Front)
        rotor3 = self.position + QTGM(self.angles, np.array([0, self.armLength, 0]))
        # direction : -y
        rotor4 = self.position + QTGM(self.angles, np.array([0, -self.armLength, 0]))

        # Format : [front rotor, otherRotors]
        return np.array([[rotor3[0], rotor3[1], rotor3[2]], [[rotor1[0], rotor2[0], rotor4[0]], [rotor1[1], rotor2[1], rotor4[1]], [rotor1[2], rotor2[2], rotor4[2]]]], dtype='object')

    def updateState(self):
        # Using PEM Model from Mr Bobby thesis
        self.t = round(self.t + self.dt, 3)
        try:
            self.x1 = -0.69 * self.x1 + 1.06 * self.inputRoll
            self.x2 = 0.8 * self.x2 - 0.24 *self.inputPitch
            self.x3 = self.x3 - 0.009 * self.inputYaw
            self.x4 = self.x4 - 0.005 *self.x5
            self.x5 = 0.005 *self.x4  + 0.91 * self.x5 - 0.003 *self.inputPitch
            self.x6 = self.x6 + 0.005 * self.x7 + 0.001 * self.inputRoll
            self.x7 = 0.003 * self.x6 + 0.91 * self.x7 - 0.1 * self.inputRoll
            self.x8 = self.x8 + 0.005 * (self.inputT)
        except:
            print('ERROR in calculate x')

        self.x1_dot = 7.86 * self.x1
        self.x2_dot = 17.1 * self.x2
        self.x3_dot = 889.7 * self.x3
        self.x4_dot = 0.78 * self.x4 - 14.4 * self.x5 
        self.x5_dot = 278.3 * self.x4 - 0.75 * self.x5 
        self.x6_dot = 0.63 * self.x6 - 4.9 * self.x7 
        self.x7_dot = -58 * self.x6 - 0.004 * self.x7 
        self.x8_dot = 11.21 * self.x8 
        
        # Update the state
        self.angles[0] = self.angles[0] + self.x1_dot * self.dt
        self.angles[1] = self.angles[1] + self.x2_dot * self.dt
        self.angles[2] = self.angles[2] + self.x3_dot * self.dt
        self.position_dot[0] = self.position_dot[0] + self.x4_dot * self.dt
        self.position[0] = self.position[0] + self.x5_dot * self.dt
        self.position_dot[1] = self.position_dot[1] + self.x6_dot * self.dt
        self.position[1] = self.position[1] + self.x7_dot * self.dt
        self.position[2] = self.position[2] + self.x8_dot * self.dt

    def controlPosition(self, positionTarget):
        self.x_err = positionTarget[0] - self.position[0]
        self.y_err = positionTarget[1] - self.position[1]
        self.z_err = positionTarget[2] - self.position[2]

        self.inputPitch = self.KP_x * self.x_err 
        + self.KI_x * self.x_err_sum
        + self.KD_x * (self.x_err - self.x_err_prev) / self.dt
        
        self.x_err_prev = self.x_err
        self.x_err_sum = self.x_err_sum + self.x_err
        
        self.inputRoll = self.KP_y * self.y_err
        + self.KI_y * self.y_err_sum
        + self.KD_y * (self.y_err - self.y_err_prev) / self.dt

        self.y_err_prev = self.y_err
        self.y_err_sum = self.y_err_sum + self.y_err

        self.inputT = self.KP_z + self.z_err
        + self.KI_z * self.z_err_sum 
        + self.KI_z * (self.z_err - self.z_err_prev) / self.dt

        if self.inputPitch > 1:
            self.inputPitch = 1
        if self.inputPitch < -1:
            self.inputPitch = -1
        if self.inputRoll > 1:
            self.inputRoll = 1
        if self.inputRoll < -1:
            self.inputRoll = -1
        if self.inputYaw > 1:
            self.inputYaw = 1
        if self.inputYaw < -1:
            self.inputYaw = -1
        if self.inputT > 1:
            self.inputT = 1
        if self.inputT < -1:
            self.inputT = -1
        
        self.updateState() 
       
    def connectToSwarmController(self, SwarmController):
        self.swarmAgent = Agent(
            self.index, (self.position[0], self.position[1], self.position[2]), self.mass)
        SwarmController.addAgent(self.swarmAgent)

    def controlSwarm(self, SwarmController):
        thisAgent = SwarmController.agents[self.index]
        totalForce = thisAgent.calculate_total_force()
        # print("FORCE", totalForce)
        thisAgent.calculateVelocity(totalForce)
        totalVelocity = thisAgent.velocity
        # print('this Agent velocity in APF ->', thisAgent.getVelocity())
        # print('Drone index', self.index)
        # print('initial position', thisAgent.position)
        thisAgent.move()
        # print('position Target', thisAgent.position)
        # print('')

        self.targetPosition = [thisAgent.position[0], thisAgent.position[1], 5]
        self.controlPosition(self.targetPosition)
        self.updateState()
        thisAgent.updatePosition(
            (self.position[0], self.position[1], self.position[2]))

        return totalVelocity