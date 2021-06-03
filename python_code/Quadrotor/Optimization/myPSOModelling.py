import numpy as np
import matplotlib.pyplot as plt
import random
import math
# from numba import njit, jit, prange
import sys
sys.path.append('../')
from Quadrotor import Quadrotor

sys.path.append('../')
from Agent import Agent
from SwarmPotentialField import SwarmPotentialField
from Target import Target
from TargetPotentialField import TargetPotentialField
from tupleUtil import calculateLength

radianToDegree = 180/math.pi
degreeToRadian = math.pi/180

class myPSOModelling(object):
    def __init__(self, simulationTime, targetOutput, min, max, responseType, realOutput):
        self.simulationTime = simulationTime
        if(responseType == "theta" or responseType == "phi" or responseType == "psi"):
            self.targetOutput = targetOutput*degreeToRadian
        self.targetOutput = targetOutput
        self.min = min
        self.max = max
        self.responseType = responseType
        self.realOutput = realOutput
        
        # For Cost Function
        self.swarmForces = []
        self.alpha = 1
        self.beta = 1

    def setCostFunctionWeight(self, alpha, beta):
        self.alpha = alpha
        self.beta = beta

    # k = [kp, ki, kd]
    def calculateQuadrotorResponse(self, k):
        specs = {"mass": 0.445, "inertia": [
            0.0027, 0.0029, 0.0053], "armLength": 0.125}
        initialState = [[0.0, 0.0, 1.0], [0.0, 0.0, 0.0],
                        [0.0, 0.0, 38.0*degreeToRadian], [0.0, 0.0, 0.0]]
        initialInput = [0.0, 0.0, 0.0, 0.0]
        attitudeControllerPID = [[1.43, 0, 0.13],  # PID phi
                                 [1.52, 0, 0.14],  # PID theta
                                 [2.43, 0, 0.26],  # PID psi
                                 [48.49, 14.29, 0]]  # PID z dot

        positionControllerPID = [[0, 0, 0],  # PID x
                                [0, 0, 0],    # PID y
                                [0, 0,  0]]  # PID z

        '''
        ANSWER

        '''
        parameterTest = {
            "phi": [0, 0],
            "theta": [0, 1],
            "psi": [0, 2],
            "zdot": [0, 3],

            "x": [1, 0],
            "y": [1, 1],
            "z": [1, 2]
        }

        index = parameterTest.get(self.responseType)
        
        controller = [attitudeControllerPID, positionControllerPID]
        controller[index[0]][index[1]] = [k[0], k[1], k[2]]

        model = Quadrotor(0, "model", specs, initialState,
                          initialInput, attitudeControllerPID, positionControllerPID)
        
        # print('state', model.position)

        targetAttitude = [0, 0, 0, 0]
        targetPosition = [0, 0, 1]
        target = [targetAttitude, targetPosition]
        target[index[0]][index[1]] = self.targetOutput

        inputPID = []
        responseValue = []

        for iteration in self.simulationTime:
            if(index[0] == 0):
                model.controlAttitude(targetAttitude)
            else:
                model.controlPosition(targetPosition)

            model.updateState()
            response = {
                "phi": model.angles[0]*radianToDegree,
                "theta": model.angles[1]*radianToDegree,
                "psi": model.angles[2]*radianToDegree,
                "zdot": model.position_dot[2],
                "x": model.position[0],
                "y": model.position[1],
                "z": model.position[2],
            }

            if response.get(self.responseType) > 100 or response.get(self.responseType) < -100:
                return responseValue, False

            responseValue.append(response.get(self.responseType, "nothing"))
        
        # print('response val', responseValue)
        return responseValue, True
    
    def calculateSwarmDrones(self, newParameters):    
        SPFParameter = newParameters[0:4]
        TPFParameter = newParameters[4:6]

        # Setup
        min_allowable_dist = self.targetOutput
        Drones = []
        position_drone1 = [(0, 0, 5)]
        Drone1 = Agent(0, position_drone1[0], 1)
        Drones.append(Drone1)

        position_drone2 = [(10, 0, 5)]
        Drone2 = Agent(1, position_drone2[0], 1)
        Drones.append(Drone2) 

        SPF = SwarmPotentialField(min_allowable_dist)
        SPF.setup(SPFParameter)

        Ship = Target([5,10,5])
        TPF = TargetPotentialField(TPFParameter[0], TPFParameter[1], 1)
        Ships = [Ship]

        responseValue = []

        for iteration in self.simulationTime:
            Drone1.SwarmPotentialForce = SPF.calculate_total_swarm_field_force(Drone1.index, Drones)
            Drone2.SwarmPotentialForce = SPF.calculate_total_swarm_field_force(Drone2.index, Drones)

            Drone1.TargetPotentialForce = TPF.calculate_target_force(Drone1.index, 0, Drones, Ships)
            Drone2.TargetPotentialForce = TPF.calculate_target_force(Drone2.index, 0, Drones, Ships)

            self.swarmForces.append(calculateLength(Drone1.calculate_total_force()))
            
            Drone1.calculateVelocity(Drone1.calculate_total_force())
            Drone1.move()
            Drone2.calculateVelocity(Drone2.calculate_total_force())
            Drone2.move()

            [distance_tuple, distance] = SPF.getDistance(0, 1, Drones) 
            responseValue.append(distance) 
            
        return responseValue, True

    def calculateMeanAbsoluteError(self, outputValues):
        error = []
        for outputValue in outputValues:
            error = np.append(error, [self.targetOutput-outputValue])
        return np.mean(abs(error))

    def calculateIntegralAbsoluteError(self, outputValues):
        error = []
        for index in range(len(outputValues)):
            error = np.append(error, [self.realOutput[index]-outputValues[index]])
        return np.sum(abs(error))

    # Spesific for APF with J = 1 * abs(Force) + 1 * abs(target distance(between quadrotors) - distance(between quadrotors))
    def calculateObjectiveFunction(self, outputValues):
        error = []
        for outputValue in outputValues:
            error = np.append(error, [self.targetOutput-outputValue])
        totalCostFunction = self.alpha * np.sum(abs(error)) + self.beta * np.sum(self.swarmForces)
        return totalCostFunction

    def particleSwarmOptimization(self, w, c1, c2, particles,  totalParameters, min_param_value, max_param_value, total_iteration, isMinimize, costFunctionType):
        costFunctionList = {
            "meanAbsoluteError": self.calculateMeanAbsoluteError,
            "integralAbsoluteError": self.calculateIntegralAbsoluteError,
            "objectiveFunction": self.calculateObjectiveFunction
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

        # Animation
        # plt.ion()
        # window = plt.figure()
        # animation = window.add_subplot(111, projection='3d')
        # animation.set_xlabel("P")
        # animation.set_ylabel("I")
        # animation.set_zlabel("D")
        # animation.set_xlim(min_param_value, max_param_value)
        # animation.set_ylim(min_param_value, max_param_value)
        # animation.set_zlim(min_param_value, max_param_value)

        # animation_pos = animation.scatter(
        #     particle_position[:, 0], particle_position[:, 1], particle_position[:, 2], color="blue")

        # window.show()

        # First loop
        for particle_index in range(particles):
            # For APF Swarm
            # [systemResponse, isStable] = self.calculateSwarmDrones(particle_position[particle_index])

            # For Attitude and Position Controller
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
        isConvergent = False
        # while (not isConvergent):
        while iterate < total_iteration :
            for particle_index in range(particles):
                # Count
                # print('BEFORE particle velocity', particle_velocity[particle_index])

                particle_velocity[particle_index] = w*particle_velocity[particle_index] + c1*random.random()*(
                    particle_position_best[particle_index]-particle_position[particle_index]) + c2*random.random()*(global_best_position-particle_position[particle_index])

                # print('AFTER particle velocity', particle_velocity[particle_index])

                particle_position[particle_index] = np.round(particle_velocity[particle_index] + \
                    particle_position[particle_index], 3)

                # For Swarm
                # [systemResponse, isStable] = self.calculateSwarmDrones(particle_position[particle_index])
                # For Attitude and Position Controller
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

            # Animation
            # plt.pause(0.1)
            # animation_pos._offsets3d = (
            #     particle_position[:, 0], particle_position[:, 1], particle_position[:, 2])
            # plt.draw()

            # Calculate position average
            position_average = np.round([np.mean(particle_position[:][0]), np.mean(particle_position[:][1]), np.mean(particle_position[:][2])],2)

            iterate = iterate+1
            print('---------------------- ITERATION ', iterate, ' ----------------------')
            print("Global best fitness: ", global_fit_value, ", Parameters :",  np.round(global_best_position, 3))
            print('Particles Position average', position_average)
            print('sum of average pos - global best pos', np.sum(np.abs(position_average - global_best_position)))
            if (np.sum(np.abs(position_average - global_best_position)) < 0.5):
                isConvergent = True

        print("Total iteration -> ", iterate)
        print("Best fitness -> ", global_fit_value)
        print("Best parameter -> ", global_best_position)
        self.bestParameter = global_best_position

    def plotBestResponse(self):
        plt.close()
    
        plt.xlabel("Time (s)")
        plt.ylabel(self.responseType+ " (degree)")

        # For APF Swarm 
        # plt.title("Distance between Drones")
        # [bestResponse, isStable] = self.calculateSwarmDrones(self.bestParameter)

        # For Attitude and Position Controller
        plt.title("Attitude Controller")
        [bestResponse, isStable] = self.calculateQuadrotorResponse(
            self.bestParameter)
        plt.plot(self.simulationTime, bestResponse, label="Model")
        plt.plot(self.simulationTime, self.realOutput, label="Real", color="red")
        plt.legend(loc='upper left')
        plt.show()
        plt.pause(100)
