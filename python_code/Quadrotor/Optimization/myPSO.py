import numpy as np
import matplotlib.pyplot as plt
import random
import math
# from numba import njit, jit, prange
import sys
sys.path.append('../')
from Quadrotor import Quadrotor

radianToDegree = 180/math.pi
degreeToRadian = math.pi/180


class myPSO(object):
    def __init__(self, simulationTime, targetOutput, min, max, responseType):
        self.simulationTime = simulationTime
        if(responseType == "theta" or responseType == "phi" or responseType == "psi"):
            self.targetOutput = targetOutput*degreeToRadian
        self.targetOutput = targetOutput
        self.min = min
        self.max = max
        self.responseType = responseType

    # k = [kp, ki, kd]
    def calculateQuadrotorResponse(self, k):
        specs = {"mass": 1.25, "inertia": [
            0.0232, 0.0232, 0.0232], "armLength": 0.265}
        initialState = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
        initialInput = [0.0, 0.0, 0.0, 0.0]
        attitudeControllerPID = [[ 1.35272881e+0, 1 0,  1.10478473e+00],  # PID phi
                                 [10.05914941, 0.0, 1.13055156],  # PID theta
                                 [0, 0, 0],  # PID psi
                                 [0, 0, 0]]  # PID z dot

        positionControllerPID = [[0, 0, 0],  # PID x
                                 [37.79524657, 29.59992759, -0.10065972],  # PID y
                                 [0, 0, 0]]  # PID z

        targetAttitude = [0, 0, 0, 0]

        parameterTest = {
            "phi": 0,
            "theta": 1,
            "psi": 2,
            "zdot": 3
        }

        attitudeControllerPID[parameterTest.get(self.responseType)] = [
            k[0], k[1], k[2]]
        targetAttitude[parameterTest.get(
            self.responseType)] = self.targetOutput
        model = Quadrotor(0, "model", specs, initialState,
                          initialInput, attitudeControllerPID, positionControllerPID)

        inputPID = []
        responseValue = []

        for iteration in self.simulationTime:
            model.controlAttitude(targetAttitude)
            model.updateState()
            response = {
                "phi": model.angles[0]*radianToDegree,
                "theta": model.angles[1]*radianToDegree,
                "psi": model.angles[2]*radianToDegree,
                "x": model.position[0],
                "y": model.position[1],
                "z": model.position[2],
                "zdot": model.position_dot[2],
            }
            if response.get(self.responseType) > 100 or response.get(self.responseType) < -100:
                return responseValue, False

            responseValue.append(response.get(self.responseType, "nothing"))
        return responseValue, True

    def calculateMeanAbsoluteError(self, outputValues):
        error = []
        for outputValue in outputValues:
            error = np.append(error, [self.targetOutput-outputValue])
        return np.mean(abs(error))

    def calculateIntegralAbsoluteError(self, outputValues):
        error = []
        for outputValue in outputValues:
            error = np.append(error, [self.targetOutput-outputValue])
        return np.sum(abs(error))

    def particleSwarmOptimization(self, w, c1, c2, particles,  totalParameters, min_param_value, max_param_value, total_iteration, isMinimize, costFunctionType):
        costFunctionList = {
            "meanAbsoluteError": self.calculateMeanAbsoluteError,
            "integralAbsoluteError": self.calculateIntegralAbsoluteError
        }

        cost_function = costFunctionList.get(costFunctionType)

        # Setup the initial conditions for position and velocity arrays
        if isMinimize:
            particle_fit_value = np.ones(particles) * 1e100
            global_fit_value = 1e100
        else:
            particle_fit_value = np.ones(particles) * 0
            global_fit_value = 0

        particle_position = np.random.uniform(
            min_param_value, max_param_value, (particles, totalParameters))
        particle_position_best = particle_position
        particle_velocity = np.zeros((particles, totalParameters))
        global_best_position = np.zeros(totalParameters)

        print('----------- PSO Algorithm starting ----------')

        plt.ion()
        window = plt.figure()
        animation = window.add_subplot(111, projection='3d')
        animation.set_xlabel("P")
        animation.set_ylabel("I")
        animation.set_zlabel("D")
        animation.set_xlim(min_param_value, max_param_value)
        animation.set_ylim(min_param_value, max_param_value)
        animation.set_zlim(min_param_value, max_param_value)

        animation_pos = animation.scatter(
            particle_position[:, 0], particle_position[:, 1], particle_position[:, 2], color="blue")

        window.show()
        # First loop
        for particle_index in range(particles):
            # Count
            [systemResponse, isStable] = self.calculateQuadrotorResponse(
                particle_position[particle_index])
            if isStable:
                cost_value = cost_function(systemResponse)
            else:
                cost_value = 1e100

            # Update
            if(particle_fit_value[particle_index] > cost_value):
                particle_fit_value[particle_index] = cost_value
                particle_position_best[particle_index] = np.copy(
                    particle_position[particle_index])

            if(global_fit_value > cost_value):
                global_fit_value = cost_value
                global_best_position = np.copy(
                    particle_position[particle_index])

        # Second loop
        iterate = 0
        while (iterate < total_iteration):
            for particle_index in range(particles):
                # Count
                particle_velocity[particle_index] = w*particle_velocity[particle_index] + c1*random.random()*(
                    particle_position_best[particle_index]-particle_position[particle_index]) + c2*random.random()*(global_best_position-particle_position[particle_index])

                particle_position[particle_index] = particle_velocity[particle_index] + \
                    particle_position[particle_index]

                [systemResponse, isStable] = self.calculateQuadrotorResponse(
                    particle_position[particle_index])
                if isStable:
                    cost_value = cost_function(systemResponse)
                else:
                    cost_value = 1e100

                # Update
                if(particle_fit_value[particle_index] > cost_value):
                    particle_fit_value[particle_index] = cost_value
                    particle_position_best[particle_index] = np.copy(
                        particle_position[particle_index])

                if(global_fit_value > cost_value):
                    global_fit_value = cost_value
                    global_best_position = np.copy(
                        particle_position[particle_index])

            plt.pause(0.1)
            animation_pos._offsets3d = (
                particle_position[:, 0], particle_position[:, 1], particle_position[:, 2])
            plt.draw()

            iterate = iterate+1
            print("Iteration: ", iterate, " -> Global best fitness: ",
                  global_fit_value, ", PID :",  global_best_position)

        print("Total iteration -> ", iterate)
        print("Best fitness -> ", global_fit_value)
        print("Best parameter -> ", global_best_position)
        self.bestParameter = global_best_position

    def plotBestResponse(self):
        plt.close()
        plt.title("Response and PID (from PSO parameter)")
        plt.xlabel("Time (s)")
        plt.ylabel("Output")
        [bestResponse, isStable] = self.calculateQuadrotorResponse(
            self.bestParameter)
        plt.plot(self.simulationTime, bestResponse, label="Best Response")
        plt.plot(self.simulationTime, self.targetOutput *
                 np.ones(self.simulationTime.shape), label="Set Point", color="red")
        plt.legend(loc='upper left')
        plt.show()
        plt.pause(100)
