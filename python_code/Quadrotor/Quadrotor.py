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

class Quadrotor(object):
    def __init__(self, index, name, specs, initialState, initialInput, attitudeControllerPID, positionControllerPID):
        self.index = index
        self.name = name
        self.t = 0  # s
        self.dt = 1/15  # s

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

        # Initiate Controlled Variable
        self.thrust = initialInput[0]  # kg m rad^2/s^2
        self.moments = [initialInput[1], initialInput[2],
                        initialInput[3]]  # kg m^2 rad^2/s^2

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

        # Swarm Control
        self.targetPosition = []

        # AR max angles
        # self.max_phi_theta_psi = 30*degreeToRadian #degree
        # For TA
        self.max_phi_theta_psi = 9*degreeToRadian 

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
        self.t = round(self.t + self.dt, 3)

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
        try:
            self.state_dot[0] = self.position_dot
            self.state_dot[1][0] = 1/self.mass*((math.sin(psi)*math.sin(
                phi)+math.cos(psi)*math.sin(theta)*math.cos(phi))*self.thrust + self.calculateFrictionForce(self.position_dot[0]))
            self.state_dot[1][1] = -1/self.mass*((-math.cos(psi)*math.sin(
                phi)+math.sin(psi)*math.sin(theta)*math.cos(phi))*self.thrust - self.calculateFrictionForce(self.position_dot[1]))
            self.state_dot[1][2] = 1/self.mass * \
                (math.cos(theta)*math.cos(phi))*self.thrust - 9.81 + \
                self.calculateFrictionForce(self.position_dot[2])/self.mass
        except:
            print('ERROR in calculate Translational Motion')

        # print('acceleration', self.state_dot[1])

        # Rotational Motion
        p = self.angles_dot[0]
        q = self.angles_dot[1]
        r = self.angles_dot[2]
        # try:
        # print('ANGLES', np.round(self.angles*radianToDegree, 2))

        self.state_dot[2][0] = p + \
            math.sin(psi)*math.tan(theta)*q+math.cos(phi)*math.tan(theta)*r
        self.state_dot[2][1] = math.cos(phi)*q-math.sin(phi)*r
        self.state_dot[2][2] = math.sin(
            phi)/math.cos(theta)*q+math.cos(phi)/math.cos(theta)*r
        # except:
            # print("ERROR in calculate Rotational Motion, Quadrotor index:", self.index)
            # print('ANGLES', self.angles)

        Ixx = self.inertia[0]
        Iyy = self.inertia[1]
        Izz = self.inertia[2]
        self.state_dot[3][0] = (Iyy-Izz)/Ixx*q*r+self.moments[0]/Ixx
        self.state_dot[3][1] = (Izz-Ixx)/Iyy*r*p+self.moments[1]/Iyy
        self.state_dot[3][2] = (Ixx-Iyy)/Izz*p*q+self.moments[2]/Izz

        # print("WTF", self.index ,self.state_dot[2][2])
        # print("HMM", phi, math.sin(
        #     phi))
        # print("CAUSE", self.state_dot[3][2])
        # if r*radianToDegree > 0 :
        #     sys.exit('WTFF')
        

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

    def controlPositionYaw(self, positionTarget, yawTarget):
        # print('---- POSITION CONTROLLER QUADROTOR- ', self.index, ' ----')

        # Yaw Control
        distanceVector = positionTarget - self.position
        distanceVal = math.sqrt(distanceVector[0]**2 + distanceVector[1]**2)
      
        ### Quad Coordinate (QC)
        quadPosQC = GTQM(self.angles, self.position)
        targetPosQC = GTQM(self.angles, positionTarget)
        distanceVectorQC = targetPosQC - quadPosQC
      
        # print('yaw_target', yawTarget)

        # print('--- GLOBAL COORDINATE ---')
        # print('quad pos', np.round(self.position, 2))
        # print('positionTarget', positionTarget, )
        # print('distanceVector', distanceVector)
        # print('distanceVal', round(distanceVal*100)/100)

        # print('--- QUADROTOR COORDINATE ---')
        # print("quadPosQC", quadPosQC)
        # print('targetPosQC', targetPosQC)
        # print('distanceVectorQC', distanceVectorQC)

        attitudeTarget = [0.0, 0.0, 0.0, 0.0]
        if abs(self.angles[2]*radianToDegree - yawTarget) < 5:
            ### Quad Coordinate (QC)
            quadPosQC = GTQM(-self.angles, self.position)
            targetPosQC = GTQM(-self.angles, positionTarget)
            distanceVectorQC = targetPosQC - quadPosQC
            self.x_err = distanceVectorQC[0]
            self.y_err = distanceVectorQC[1]

            # print('X ERR - Y ERR', self.x_err, self.y_err)

            attitudeTarget[0] = self.KP_y * self.y_err
            + self.KI_y * self.y_err_sum
            + self.KD_y * (self.y_err - self.y_err_prev) / self.dt

            self.y_err_prev = self.y_err
            self.y_err_sum = self.y_err_sum + self.y_err

            attitudeTarget[0] = attitudeTarget[0]

            attitudeTarget[1] = self.KP_x * self.x_err
            + self.KI_x * self.x_err_sum
            + self.KD_x * (self.x_err - self.x_err_prev) / self.dt
       
            self.x_err_prev = self.x_err
            self.x_err_sum = self.x_err_sum + self.x_err

            attitudeTarget[1] = attitudeTarget[1]

            if attitudeTarget[0] > self.max_phi_theta_psi:
                attitudeTarget[0] = self.max_phi_theta_psi
            if attitudeTarget[0] < -self.max_phi_theta_psi:
                attitudeTarget[0] = -self.max_phi_theta_psi
            if attitudeTarget[1] > self.max_phi_theta_psi:
                attitudeTarget[1] = self.max_phi_theta_psi
            if attitudeTarget[1] < -self.max_phi_theta_psi:
                attitudeTarget[1] = -self.max_phi_theta_psi
    
        attitudeTarget[2] = yawTarget*degreeToRadian
        if attitudeTarget[2] - self.angles[2] > self.max_phi_theta_psi:
            attitudeTarget[2] = self.angles[2] + self.max_phi_theta_psi
        if attitudeTarget[2] - self.angles[2] < -self.max_phi_theta_psi:
            attitudeTarget[2] = self.angles[2] - self.max_phi_theta_psi
       
        attitudeTarget[3] = self.KP_z * self.z_err
        + self.KI_z * self.z_err_sum
        + self.KD_z * (self.z_err - self.z_err_prev) / self.dt

        self.z_err_prev = self.z_err
        self.z_err_sum = self.z_err_sum + self.z_err

        # print('Current Angles', np.round(np.array(self.angles)*radianToDegree, 2))
        # print("ATTITUDE TARGET", np.array(attitudeTarget) * radianToDegree)
        self.controlAttitude(np.array(attitudeTarget)*radianToDegree)
        # print()

    def controlPosition(self, positionTarget):
        
        attitudeTarget = np.array([0.0,0.0,0.0,0.0])  # psi theta phi zdot (degree and m/s)
        # print('------------------------------')
        # print('---- INITIAL ATTITUDE TARGET QUADROTOR- ', self.index, ' ----')

        # Yaw Control
        distanceVector = positionTarget - self.position
        distanceVal = math.sqrt(distanceVector[0]**2 + distanceVector[1]**2)
      
        yaw_target = np.arctan2([distanceVector[1]], [distanceVector[0]])[0]
        # yaw_target = math.atan(distanceVector[1]/distanceVector[0])
        psi_err =  yaw_target - self.angles[2]

        ### Quad Coordinate (QC)
        quadPosQC = GTQM(self.angles, self.position)
        targetPosQC = GTQM(self.angles, positionTarget)
        distanceVectorQC = targetPosQC - quadPosQC
        
        # print('---- POSITION CONTROLLER QUADROTOR- ', self.index, ' ----')
        # print('yaw_target', yaw_target*radianToDegree)
        # print('psi_err', psi_err*radianToDegree)
        # print('quad pos', np.round(self.position, 2))
        # print('positionTarget, self.position', positionTarget, self.position)
        # print('distanceVector', distanceVector)
        # print('distanceVal', round(distanceVal*100)/100)

        # print("quadPosQC", quadPosQC)
        # print('targetPosQC', targetPosQC)
        # print('distanceVectorQC', distanceVectorQC)

        self.x_err = distanceVectorQC[0]
        self.y_err = distanceVectorQC[1]    

        # print('hm', self.x_err < 0.1, self.x_err > -0.1, self.y_err < 0.1, self.y_err > -0.1)
        # print('hm2', self.x_err < 0.1 and self.x_err > -0.1 and self.y_err < 0.1 and self.y_err > -0.1)

        if (psi_err*radianToDegree > 0.01 or psi_err*radianToDegree < -0.01) and distanceVal > 0.5:
            self.yaw_target = yaw_target
        else:
            attitudeTarget[0] = self.KP_y * self.y_err
            + self.KI_y * self.y_err_sum
            + self.KD_y * (self.y_err - self.y_err_prev) / self.dt

            self.y_err_prev = self.y_err
            self.y_err_sum = self.y_err_sum + self.y_err

            attitudeTarget[1] = self.KP_x * self.x_err
            + self.KI_x * self.x_err_sum
            + self.KD_x * (self.x_err - self.x_err_prev) / self.dt
       
            # attitudeTarget[1] = attitudeTarget[1] * -1
            self.x_err_prev = self.x_err
            self.x_err_sum = self.x_err_sum + self.x_err

        attitudeTarget[2] = self.yaw_target

        self.z_err = positionTarget[2] - self.position[2]

        attitudeTarget[3] = self.KP_z * self.z_err
        + self.KI_z * self.z_err_sum
        + self.KD_z * (self.z_err - self.z_err_prev) / self.dt

        self.z_err_prev = self.z_err
        self.z_err_sum = self.z_err_sum + self.z_err
        
        # print('---- ATTITUDE QUADROTOR- ', self.index, ' ----')
        # print('Current Angles', np.round(np.array(self.angles)*radianToDegree, 2))
        # print('Before Attitude Target', np.round(np.array(attitudeTarget)*radianToDegree, 2))
        if attitudeTarget[0] > self.max_phi_theta_psi:
            attitudeTarget[0] = self.max_phi_theta_psi
        if attitudeTarget[0] < -self.max_phi_theta_psi:
            attitudeTarget[0] = -self.max_phi_theta_psi
        if attitudeTarget[1] > self.max_phi_theta_psi:
            attitudeTarget[1] = self.max_phi_theta_psi
        if attitudeTarget[1] < -self.max_phi_theta_psi:
            attitudeTarget[1] = -self.max_phi_theta_psi
        if attitudeTarget[2] - self.angles[2] > self.max_phi_theta_psi:
            attitudeTarget[2] = self.angles[2] + self.max_phi_theta_psi
        if attitudeTarget[2] - self.angles[2] < -self.max_phi_theta_psi:
            attitudeTarget[2] = self.angles[2] - self.max_phi_theta_psi
        # print('After Attitude Target', np.round(np.array(attitudeTarget)*radianToDegree, 2))
        # print('zdot target', attitudeTarget[3])
        self.controlAttitude(np.array(attitudeTarget)*radianToDegree)
        return [self.KP_x * self.x_err, self.x_err, self.KI_x * self.x_err_sum, self.x_err_sum, self.KD_x * (self.x_err - self.x_err_prev) / self.dt, (self.x_err - self.x_err_prev) / self.dt, attitudeTarget[1], self.x_err]

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