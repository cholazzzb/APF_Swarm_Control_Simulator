import math
import numpy as np


class Quadrotor(object):
    def __init__(self, specs, initialState, initialInput, attitudeControllerPID):
        self.t = 0
        self.dt = 0.01

        self.mass = specs["mass"]
        self.inertia = specs["inertia"]  # [Ixx, Iyy, Izz]

        # Quadrotor Initial State
        self.position = np.array(initialState[0])
        self.position_dot = np.array(initialState[1])
        self.angles = np.array(initialState[2])  # [phi theta psi]
        self.angles_dot = np.array(initialState[3])
        self.state_dot = np.array([[0, 0, 0],  # position_dot
                                   [0, 0, 0],  # position_dot_dot
                                   [0, 0, 0],  # angles_dot
                                   [0, 0, 0]])  # angles_dot_dot

        # Initiate Controlled Variable
        self.torque = initialInput[0]
        self.moment = [initialInput[1], initialInput[2], initialInput[3]]

        # Attitude Controller with PID
        self.KP_phi = attitudeControllerPID[1][0]
        self.KI_phi = attitudeControllerPID[1][1]
        self.KD_phi = attitudeControllerPID[1][2]

        self.KP_theta = attitudeControllerPID[2][0]
        self.KI_theta = attitudeControllerPID[2][1]
        self.KD_theta = attitudeControllerPID[2][2]

        self.KP_psi = attitudeControllerPID[3][0]
        self.KI_psi = attitudeControllerPID[3][1]
        self.KD_psi = attitudeControllerPID[3][2]

        self.phi_err = 0
        self.phi_err_prev = 0
        self.phi_err_sum = 0

        self.theta_err = 0
        self.theta_err_prev = 0
        self.theta_err_sum = 0

        self.psi_err = 0
        self.psi_err_prev = 0
        self.psi_err_sum = 0

        self.KP_zdot = attitudeControllerPID[0][0]
        self.KI_zdot = attitudeControllerPID[0][1]
        self.KD_zdot = attitudeControllerPID[0][2]

        self.zdot_err = 0
        self.zdot_err_prev = 0
        self.zdot_err_sum = 0

    def getState(self):
        return [self.position, self.position_dot, self.angles, self.angles_dot]

    def updateState(self):
        self.t = self.t + self.dt

        # Evaluate Equation of Motions
        phi = self.angles[0]
        theta = self.angles[1]
        psi = self.angles[2]

        # Translational Motion
        self.state_dot[0] = self.position_dot
        self.state_dot[1][0] = -1/self.mass*(math.sin(psi)*math.sin(
            phi)+math.cos(psi)*math.sin(theta)*math.cos(phi))*self.torque
        self.state_dot[1][1] = -1/self.mass*(-math.cos(psi)*math.sin(
            phi)+math.sin(psi)*math.sin(theta)*math.cos(phi))*self.torque
        self.state_dot[1][2] = -1/self.mass * \
            (math.cos(theta)*math.cos(phi))*self.torque + 9.81

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
        self.state_dot[3][0] = (Iyy-Izz)/Ixx*q*r+self.moment[0]/Ixx
        self.state_dot[3][1] = (Izz-Ixx)/Iyy*r*p+self.moment[1]/Iyy
        self.state_dot[3][2] = (Ixx-Iyy)/Izz*p*q+self.moment[2]/Izz

        # Update the state
        self.position = self.position + self.state_dot[0] * self.dt
        self.position_dot = self.position_dot + self.state_dot[1]*self.dt
        self.angles = self.angles + self.state_dot[2] * self.dt
        self.angles_dot = self.angles_dot + self.state_dot[3] * self.dt

        # Add sensor noise with white noise

    def controlAttitude(self, attitudeTarget):
        self.phi_err = attitudeTarget[1] - self.angles[0]
        self.theta_err = attitudeTarget[2] - self.angles[1]
        self.psi_err = attitudeTarget[3] - self.angles[2]

        # Calculate output for controlled variable
        '''
        self.moment[0] = phi
        self.moment[1] = theta
        self.moment[2] = psi
        self.torque = position -> z_dot
        '''
        self.moment[0] = self.KP_phi * self.phi_err + self.KI_phi*self.phi_err_sum + \
            self.KD_phi * (self.phi_err - self.phi_err_prev)/self.dt

        self.moment[1] = self.KP_theta * self.theta_err + self.KI_theta*self.theta_err_sum + \
            self.KD_theta * (self.theta_err - self.theta_err_prev)/self.dt

        self.moment[2] = self.KP_psi * self.psi_err + self.KI_psi*self.psi_err_sum + \
            self.KD_psi * (self.psi_err - self.psi_err_prev)/self.dt

        self.phi_err_prev = self.phi_err
        self.phi_err_sum = self.phi_err_sum + self.phi_err
        self.theta_err_prev = self.theta_err
        self.theta_err_sum = self.theta_err_sum + self.theta_err
        self.psi_err_prev = self.psi_err
        self.psi_err_sum = self.psi_err_sum + self.psi_err

        self.zdot_err = attitudeTarget[0] - self.position_dot[2]
        self.torque = self.mass * 9.81 - (self.KP_zdot * self.zdot_err + self.KI_zdot *
                                      self.zdot_err_sum + self.KD_zdot*(self.zdot_err - self.zdot_err_prev)/self.dt)
        self.zdot_err_prev = self.zdot_err
        self.zdot_err_sum = self.zdot_err_sum + self.zdot_err
