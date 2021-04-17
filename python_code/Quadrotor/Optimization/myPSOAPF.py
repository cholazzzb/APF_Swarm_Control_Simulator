import numpy as np
import matplotlib.pyplot as plt
import random
import math
# from numba import njit, jit, prange
import sys
sys.path.append('../')
from Quadrotor import Quadrotor
from Ship import Ship
from Bird import Bird

sys.path.append('../')
from Agent import Agent
from SwarmController import SwarmController
from SwarmPotentialField import SwarmPotentialField
from Target import Target
from TargetPotentialField import TargetPotentialField
from tupleUtil import calculateLength
from tupleUtil import minusWithTuple

radianToDegree = 180/math.pi
degreeToRadian = math.pi/180

class myPSOAPF(object):
    def __init__(self, simulationTime, targetOutput, min, max, responseType):
        self.simulationTime = simulationTime
        if(responseType == "theta" or responseType == "phi" or responseType == "psi"):
            self.targetOutput = targetOutput*degreeToRadian
        self.targetOutput = targetOutput
        self.min = min
        self.max = max
        self.responseType = responseType
        
        # For Cost Function
        self.totalVelocity = []
        self.averageTargetSwarmDistance = []
        self.alpha = 1
        self.beta = 1
        self.gamma = 1

    def setCostFunctionWeight(self, alpha, beta, gamma):
        self.alpha = alpha
        self.fitnessAlpha = 0
        self.bestFitnessAlpha = 0
        self.beta = beta
        self.fitnessBeta = 0
        self.bestFitnessBeta = 0
        self.gamma = gamma
        self.fitnessGamma = 0
        self.bestFitnessGamma = 0

    # k = [kp, ki, kd]
    def calculateQuadrotorResponse(self, k):
        specs = {"mass": 0.445, "inertia": [
            0.0027, 0.0029, 0.0053], "armLength": 0.125}
        initialState = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
        initialInput = [0.0, 0.0, 0.0, 0.0]
        attitudeControllerPID = [[1.43, 0, 0.13],  # PID phi
                                 [1.52, 0, 0.14],  # PID theta
                                 [2.43, 0, 0.26],  # PID psi
                                 [88.02, 44.5, 0]]  # PID z dot

        positionControllerPID = [[0, 0, 0 ],  # PID x
                                [0, 0, 0],  # PID y
                                [0, 0, 0]]  # PID z

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
        
        targetAttitude = [0, 0, 0, 0]
        targetPosition = [0, 0, 0]
        target = [targetAttitude, targetPosition]
        target[index[0]][index[1]] = self.targetOutput

        inputPID = []
        responseValue = []

        for iteration in self.simulationTime:
            if(index[0] == 0):
                model.controlAttitude(targetAttitude)
            else:
                model.controlPosition(targetPosition)

            try:
                model.updateState()
            except:
                print("NOT STABLE")
                return responseValue, False

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
        return responseValue, True
    
    def calculateSwarmDrones(self, newParameters):  
        # min_allowable_dist = self.targetOutput
        min_allowable_dist = 1  

        SPFParameter = newParameters[0:4]
        SPFConfig = {
            "min_allowable_dist": min_allowable_dist
        }
        TPFConfig = {
            "damping_factor": newParameters[4], 
            "gain":newParameters[5], 
            "target_detecting_range":1
        }
        OPFConfig = {
            "positiveGain1": newParameters[6], 
            "positiveGain2":newParameters[7], 
            "detecting_range": 1
            }

        # Setup
        specs = {"mass": 0.445, "inertia": [
            0.0027, 0.0029, 0.0053], "armLength": 0.125}
        initialInput = [0.0, 0.0, 0.0, 0.0]
        attitudeControllerPID = [[1.43, 0, 0.13],  # PID phi
                                    [1.52, 0, 0.14],  # PID theta
                                    [2.43, 0, 0.26],  # PID psi
                                    [48.49, 14.29, 0.0]]  # PID z dot

        positionControllerPID = [[323.35, 8.38, 177.91],  # PID x
                                [138.38, 126.43, 98.03],    # PID y
                                [4.31, 8.1,  8.44]]  # PID z

        Drones = []
        position_drone1 = (1.1, 5, 5)
        Drone1 = Agent(0, position_drone1, 1)
        Drones.append(Drone1)
        initialState1 = [[1.0, 5.0, 5.0], [0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]

        AR1 = Quadrotor(0, "AR1", specs, initialState1, initialInput, attitudeControllerPID, positionControllerPID)

        position_drone2 = (0, 5, 5)
        Drone2 = Agent(1, position_drone2, 1)
        Drones.append(Drone2)
        initialState2 = [[0.0, 5.0, 5.0], [0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]

        AR2 = Quadrotor(0, "AR2", specs, initialState2, initialInput, attitudeControllerPID, positionControllerPID)

        Bird1 = Bird([2, 4.7, 5])
        Ship1 = Ship([3,5,5], 1)

        SwarmController1 = SwarmController(TPFConfig, OPFConfig, SPFConfig)
        SwarmController1.configureSPF(SPFParameter)

        SwarmController1.addAgent(Drone1)
        SwarmController1.addAgent(Drone2)
        SwarmController1.addObstacle(Bird1)

        # Connect 
        AR1.connectToSwarmController(SwarmController1)
        AR2.connectToSwarmController(SwarmController1)
        Bird1.connectToSwarmController(SwarmController1)
        Ship1.connectToSwarmController(SwarmController1)

        responseValue = []
        self.totalVelocity = []
        self.averageTargetSwarmDistance = []

        min_disTargetSwarm = 100000
        for iteration in self.simulationTime:
            SwarmController1.calculateAgentsForces()
            try:
                totalVelocity1 = AR1.controlSwarm(SwarmController1)
                totalVelocity2 = AR2.controlSwarm(SwarmController1)
            except:
                print("NOT STABLE")
                return responseValue, False 

            [distance_tuple, distance] = SwarmController1.SPF.getDistance(0, 1, Drones) 
            # print('distanceBetween', distance)
            responseValue.append(distance) 
            # print("VELOCITY", calculateLength(totalVelocity1) + calculateLength(totalVelocity2))
            self.totalVelocity.append(abs(calculateLength(totalVelocity2)) + abs(calculateLength(totalVelocity2)))
            targetPosition = (3,5,5)
            tupleDistance1 = minusWithTuple(targetPosition, (AR1.position[0], AR1.position[1], AR1.position[2]))
            tupleDistance2 = minusWithTuple(targetPosition, (AR2.position[0], AR2.position[1], AR2.position[2]))
            averageDistance = math.sqrt(abs(sum(tuple(pow(x, 2) for x in tupleDistance1)))) + math.sqrt(abs(sum(tuple(pow(x, 2) for x in tupleDistance2))))
            # print('dis target swarms', averageDistance)
            if averageDistance < min_disTargetSwarm:
                min_disTargetSwarm = averageDistance
            self.averageTargetSwarmDistance.append(averageDistance)
        print('min dis target swarm', min_disTargetSwarm)
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

    # Spesific for APF with J = 1 * abs(Force) + 1 * abs(target distance(between quadrotors) - distance(between quadrotors))
    def calculateObjectiveFunction(self, outputValues):
        error = []
        for outputValue in outputValues:
            error = np.append(error, [abs(self.targetOutput-outputValue)])
            # print('distance dev', [abs(self.targetOutput-outputValue)])
        self.fitnessAlpha = self.alpha * np.sum(abs(error)) # distance between quadrotor
        self.fitnessBeta = self.beta * np.sum(self.totalVelocity) / 2
        self.fitnessGamma = self.gamma * np.sum(self.averageTargetSwarmDistance) / 2
        return self.fitnessAlpha + self.fitnessBeta + self.fitnessGamma

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

        particle_position = np.round(np.random.uniform(
            min_param_value, max_param_value, (particles, totalParameters)),2)
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
            [systemResponse, isStable] = self.calculateSwarmDrones(particle_position[particle_index])

            # For Attitude and Position Controller
            # [systemResponse, isStable] = self.calculateQuadrotorResponse(
            #     particle_position[particle_index])
            if isStable:
                cost_value = cost_function(systemResponse)
            else:
                cost_value = 1e100

            print('CF', cost_value)

            # Update
            if(particle_fit_value[particle_index] > cost_value):
                particle_fit_value[particle_index] = cost_value
                particle_position_best[particle_index] = np.copy(
                    particle_position[particle_index])

            if(global_fit_value > cost_value):
                global_fit_value = cost_value
                self.bestFitnessAlpha = self.fitnessAlpha
                self.bestFitnessBeta = self.fitnessBeta   
                self.bestFitnessGamma = self.fitnessGamma
                global_best_position = np.copy(
                    particle_position[particle_index])

        # Second loop
        iterate = 0
        while (iterate < total_iteration):
            for particle_index in range(particles):
                # Count
                particle_velocity[particle_index] = w*particle_velocity[particle_index] + c1*random.random()*(
                    particle_position_best[particle_index]-particle_position[particle_index]) + c2*random.random()*(global_best_position-particle_position[particle_index])

                particle_position[particle_index] = np.round(particle_velocity[particle_index] + \
                    particle_position[particle_index], 2)

                # For Swarm
                [systemResponse, isStable] = self.calculateSwarmDrones(particle_position[particle_index])
                # For Attitude and Position Controller
                # [systemResponse, isStable] = self.calculateQuadrotorResponse(
                #     particle_position[particle_index])
                if isStable:
                    cost_value = cost_function(systemResponse)
                else:
                    cost_value = 1e100

                print('CF', cost_value)
                # Update
                if(particle_fit_value[particle_index] > cost_value):
                    particle_fit_value[particle_index] = cost_value
                    particle_position_best[particle_index] = np.copy(
                        particle_position[particle_index])

                if(global_fit_value > cost_value):
                    global_fit_value = cost_value
                    self.bestFitnessAlpha = self.fitnessAlpha
                    self.bestFitnessBeta = self.fitnessBeta
                    self.bestFitnessGamma = self.fitnessGamma
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
            print("Alpha fitness ->", self.bestFitnessAlpha)
            print("Beta fitness ->", self.bestFitnessBeta)
            print("Gamma fitness ->", self.bestFitnessGamma)
  

        print("Total iteration -> ", iterate)
        print("Best fitness -> ", global_fit_value)
        print("Alpha fitness ->", self.bestFitnessAlpha)
        print("Beta fitness ->", self.bestFitnessBeta)
        print("Gamma fitness ->", self.bestFitnessGamma)
        print("Best parameter -> ", global_best_position)
        self.bestParameter = global_best_position

    def plotBestResponse(self):
        plt.close()
    
        plt.xlabel("Time (s)")
        plt.ylabel("Output")

        # For APF Swarm 
        plt.title("Distance between Drones")
        [bestResponse, isStable] = self.calculateSwarmDrones(self.bestParameter)

        # For Attitude and Position Controller
        # plt.title("Response and PID (from PSO parameter)")
        # [bestResponse, isStable] = self.calculateQuadrotorResponse(
            # self.bestParameter)
        plt.plot(self.simulationTime, bestResponse, label="Best Response")
        plt.plot(self.simulationTime, self.targetOutput *
                 np.ones(self.simulationTime.shape), label="Set Point", color="red")
        plt.legend(loc='upper left')
        plt.show()
        plt.pause(100)
