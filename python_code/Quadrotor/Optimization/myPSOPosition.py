import numpy as np
import matplotlib.pyplot as plt
import random
import math
# from numba import njit, jit, prange
import sys
sys.path.append('../')
from QuadrotorARSim import QuadrotorARSim

sys.path.append('../')
from Agent import Agent
from SwarmPotentialField import SwarmPotentialField
from Target import Target
from TargetPotentialField import TargetPotentialField
from tupleUtil import calculateLength

radianToDegree = 180/math.pi
degreeToRadian = math.pi/180

class myPSOPosition(object):
    def __init__(self, simulationTime, targetOutput, min, max):
        self.simulationTime = simulationTime
        self.targetOutput = targetOutput
        self.min = min
        self.max = max
        
        # For Cost Function
        self.swarmForces = []
        self.alpha = 1
        self.beta = 1

    def setCostFunctionWeight(self, alpha, beta):
        self.alpha = alpha
        self.beta = beta

    # k = [KPtheta, KItheta, KDtheta, KPx, KIx, KDx, 
    #       KPphi, KIphi, KDphi, KPy, KIy, KDy, 
    #       KPz, KIz, KDz,
    #       KPpsi, KIpsi, KDpsi]

    def calculateQuadrotorResponse(self, k):
        # print('k', k)
        specs = {"mass": 0.445, "inertia": [
            0.0027, 0.0029, 0.0053], "armLength": 0.125}
        initialState = [[0.0, 0.0, 1.0], [0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
        initialInput = [0.0, 0.0, 0.0, 0.0]
        attitudeControllerPID = [[k[6], k[7], k[8]],  # PID phi
                                 [k[0], k[1], k[2]],  # PID theta
                                 [0, 0, 0],  # PID psi
                                 [0, 0, 0]]  # PID z dot

        positionControllerPID = [[k[3], k[4], k[5]],  # PID x
                                 [k[9], k[10], k[11]],  # PID y
                                [k[12], k[13], k[14]]]  # PID z
        
        model = QuadrotorARSim(0, "model", specs, initialState,
                          initialInput, attitudeControllerPID, positionControllerPID)
        
        targetIndex = 0

        inputPID = []
        responseValue = []
        errorValues = []

        isStable = True

        for iteration in self.simulationTime:
            # print('target Index', targetIndex)
            # model.controlPosition(self.targetOutput[targetIndex])
            model.controlPositionAjmera(self.targetOutput[targetIndex])
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

            # print('z', response.get('z'))
            # print('theta', response.get('theta'))
            # print('moments', model.moments)
            # if (response.get("z") < -100 or response.get("z") > 100): 
            if (response.get("x") > 100 or response.get("x") < -100 or response.get("y") > 100 or response.get("y") < -100 or response.get("z") > 100 or response.get("z") < -100 or response.get("phi") > 100 or response.get("phi") < -100 or response.get("theta") > 100 or response.get("theta") < -100):
                
                # print('moments',model.moments)
                # print('checker theta', response.get('theta'))
                # print('checker', response.get("theta") > 100 or response.get("theta") < -100)
                isStable = False
                break
                # return responseValue, errorValues, False

            responseValue.append([response.get("x"), response.get("y"), response.get("z")])
            errorValues.append(abs(response.get("x") - self.targetOutput[targetIndex][0]) + abs(response.get("y") - self.targetOutput[targetIndex][1]) + abs(response.get("z") - self.targetOutput[targetIndex][2]))
            # print('error', abs(response.get("x") - self.targetOutput[targetIndex][0]) + abs(response.get("y") - self.targetOutput[targetIndex][1]) + abs(response.get("z") - self.targetOutput[targetIndex][2]))
            # print('errorx', abs(response.get("y") - self.targetOutput[targetIndex][1]))

            # if (abs(response.get("x") - self.targetOutput[targetIndex][0]) + abs(response.get("y") - self.targetOutput[targetIndex][1]) + abs(response.get("z") - self.targetOutput[targetIndex][2])) < 0.3 and targetIndex < len(self.targetOutput) - 1:
            #     targetIndex = targetIndex + 1

        # print('errV', errorValues)
        # print('err', np.sum(errorValues))
        return np.array(responseValue), errorValues, isStable
    
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

    def calculateIntegralAbsoluteError(self, errorValue):
        return np.sum(errorValue)

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
            [systemResponse, errorValues, isStable] = self.calculateQuadrotorResponse(
                particle_position[particle_index])

            # print('IS STABLE', isStable)
            if isStable:
                cost_value = cost_function(errorValues)
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

                # print('vel', particle_velocity)

                particle_position[particle_index] = np.round(particle_velocity[particle_index] + \
                    particle_position[particle_index], 2)

                particle_position[particle_position < 0] = 0

                # For Swarm
                # [systemResponse, isStable] = self.calculateSwarmDrones(particle_position[particle_index])
                # For Attitude and Position Controller
                [systemResponse, errorValues, isStable] = self.calculateQuadrotorResponse(
                    particle_position[particle_index])
                # print('IS STABLE', isStable)
                # print("ERRRR", errorValues)
                if isStable:
                    cost_value = cost_function(errorValues)
                else:
                    cost_value = 1e100

                # Update

                # Check current particle fit value and cost value to debug the error
                # print('----------')
                # print('particle fit value', particle_fit_value[particle_index])
                # print('cost value', cost_value, particle_position[particle_index])
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

            iterate = iterate+1
            print("Iteration: ", iterate, " -> Global best fitness: ",
                  global_fit_value, ", Parameters :",  global_best_position)

        print("Total iteration -> ", iterate)
        print("Best fitness -> ", global_fit_value)
        print("Best parameter -> ", global_best_position)
        self.bestParameter = global_best_position

    def plotBestResponse(self):
        plt.close()
    
        plt.xlabel("Time (s)")
        plt.ylabel("Output")

        # For APF Swarm 
        # plt.title("Distance between Drones")
        # [bestResponse, isStable] = self.calculateSwarmDrones(self.bestParameter)

        # For Attitude and Position Controller
        plt.title("Best Response and PID (from PSO parameter)")
        [bestResponse, errorValues, isStable] = self.calculateQuadrotorResponse(
            self.bestParameter)
        plt.plot(self.simulationTime, bestResponse[:, 0], label="x Pos", color="red")
        plt.plot(self.simulationTime, bestResponse[:, 1], label="y Pos", color="yellow")
        plt.plot(self.simulationTime, bestResponse[:, 2], label="z Pos", color="green")

        plt.plot(self.simulationTime, self.targetOutput[0][0] *
                 np.ones(self.simulationTime.shape), label="Set Point x", color="black")
        plt.plot(self.simulationTime, self.targetOutput[0][1] *
                 np.ones(self.simulationTime.shape), label="Set Point y", color="purple")
        plt.plot(self.simulationTime, self.targetOutput[0][2] *
                 np.ones(self.simulationTime.shape), label="Set Point z", color="blue")

        plt.legend(loc='lower right')
        plt.show()
        plt.pause(100)
