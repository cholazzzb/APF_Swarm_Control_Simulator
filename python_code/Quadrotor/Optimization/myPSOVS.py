import numpy as np
import matplotlib.pyplot as plt
import random
import math
# from numba import njit, jit, prange
import sys
sys.path.append('../')
from Quadrotor import Quadrotor
from VirtualStructure import VirtualStructure
from Bird import Bird
from Ship import Ship

sys.path.append('../')
from Agent import Agent
from SwarmPotentialField import SwarmPotentialField
from Target import Target
from TargetPotentialField import TargetPotentialField
from SwarmController import SwarmController
from tupleUtil import calculateLength

radianToDegree = 180/math.pi
degreeToRadian = math.pi/180

class myPSOVS(object):
    def __init__(self, simulationTime, targetOutput, min, max, responseType):
        self.simulationTime = simulationTime
        if(responseType == "theta" or responseType == "phi" or responseType == "psi"):
            self.targetOutput = targetOutput*degreeToRadian
        self.targetOutput = targetOutput
        self.min = min
        self.max = max
        self.responseType = responseType
        
        # For Cost Function
        self.swarmForces = []
        self.alpha = 1
        self.beta = 1

    def setCostFunctionWeight(self, alpha, beta):
        self.alpha = alpha
        self.beta = beta
    
    def calculateVSOnly(self, k):
        TPFconfig = {"damping_factor": k[0], "gain":k[1], "target_detecting_range":1}
        OPFconfig = {"positiveGain1": k[2], "positiveGain2":k[3], "detecting_range": 2}
        SPFconfig = {"min_allowable_dist": 10}
        SwarmController1 = SwarmController(TPFconfig, OPFconfig, SPFconfig)

        # Virtual Structure
        VS = VirtualStructure("triangle", (0,0,0*degreeToRadian))
        initialVSPos = [0.0,0.0,5.0]
        VS.setFRPPosition(initialVSPos)
        VS.connectToSwarmController(SwarmController1)

        Target1 = Ship([10, 10, 5], 5)
        Target1.connectToSwarmController(SwarmController1)

        ObstaclesPosition = [
            [5,6,5], [5,5,5], [5,4,5], [5,9,5],
            [4,2,5], [7,9,5], [8,9,5], [0,2,5]]
        
        for obstaclePosition in ObstaclesPosition:
            ObstacleObject = Bird(obstaclePosition)
            ObstacleObject.connectToSwarmController(SwarmController1)

        VSPositionHistory = np.array([VS.FRPPosition])

        for iteration in range(100):

            try:
                SwarmController1.calculateAgentsForces()
                APFForce = SwarmController1.agents[0].calculate_total_force()
                VS.moveFRP(APFForce)

                newTargetPos = VS.VSPPositions 
                VSPositionHistory = np.concatenate(
                    (VSPositionHistory, np.array([
                        VS.FRPPosition
                    ]))
                )
                # print('------------------------------')
                # print('---- VS - REAL WORLD POSITION')
                # print("APF Force", APFForce)
                # print("VS RP", VS.FRPPosition)
                # print('NEW TARGET POS 1', newTargetPos[0])
                # print('NEW TARGET POS 2', newTargetPos[1])
                # print('NEW TARGET POS 3', newTargetPos[2])
                # print('')
                # print('------------------------------')
            except: 
                # print("VS.FRPPosition, VS HIstory Pos", VS.FRPPosition, VSPositionHistory[-1])
                # print("NABRAK WKWKKW")
                return VSPositionHistory, False

            # if collide return False
            # try:
            #     for obstaclePosition in ObstaclesPosition:
            #         distanceWithObstacle = abs(np.array(VS.FRPPosition) - np.array(obstaclePosition))
            #         # print(distanceWithObstacle)
            #         # print(math.sqrt(distanceWithObstacle[0]**2 + distanceWithObstacle[1]**2 + distanceWithObstacle[2]**2))
            #         if math.sqrt(distanceWithObstacle[0]**2 + distanceWithObstacle[1]**2 + distanceWithObstacle[2]**2) < 1:
            #             print("NABRAKK")
            #             raise Exception("NABRAKKK")
            # except:
            #         return VSPositionHistory, False
        # print("VS.FRPPosition, VS HIstory Pos", VS.FRPPosition, VSPositionHistory[-1])
        return VSPositionHistory, True

    def calculateVS(self, k):
        TPFconfig = {"damping_factor": k[0], "gain":k[1], "target_detecting_range":1}
        OPFconfig = {"positiveGain1": k[2], "positiveGain2":k[3], "detecting_range": 2}
        SPFconfig = {"min_allowable_dist": 10}
        SwarmController1 = SwarmController(TPFconfig, OPFconfig, SPFconfig)

        # Virtual Structure
        VS = VirtualStructure("triangle", (0,0,0*degreeToRadian))
        initialVSPos = [0.0,0.0,5.0]
        VS.setFRPPosition(initialVSPos)
        VS.connectToSwarmController(SwarmController1)

        # Build Quadrotor
        specs = {"mass": 0.445, "inertia": [
            0.0027, 0.0029, 0.0053], "armLength": 0.125}
        initialState = [[0.0, 0.0, 5.0], [0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
        initialState2 = [[-0.5, -0.5, 5.0], [0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
        initialState3 = [[-1.5, -0.5, 5.0], [0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]

        initialInput = [0.0, 0.0, 0.0, 0.0]
        attitudeControllerPID = [[1.43, 0, 0.13],  # PID phi
                                [1.52, 0, 0.14],  # PID theta
                                [2.43, 0, 0.26],  # PID psi
                                [48.49, 14.29, 0.0]]  # PID z dot
        positionControllerPID = [[1.59, 56.7, 4.77],  # PID x
                                [0.96, 88.38, 73.36],    # PID y
                                [0.08, 40.31, 92.08]]  # PID z

        AR1 = Quadrotor(0, "AR1", specs, initialState,
                        initialInput, attitudeControllerPID, positionControllerPID)
        AR2 = Quadrotor(1, "AR2", specs, initialState2,
                        initialInput, attitudeControllerPID, positionControllerPID)
        AR3 = Quadrotor(2, "AR3", specs, initialState3,
                        initialInput, attitudeControllerPID, positionControllerPID)

        Target1 = Ship([10, 10, 0], 5)
        Target1.connectToSwarmController(SwarmController1)

        ObstaclesPosition = [
            [5,6,5], [5,5,5], [5,4,5], [5,9,5], 
            [4,2,5], [7,9,5], [8,9,5], [2,0,5]]
        
        for obstaclePosition in ObstaclesPosition:
            ObstacleObject = Bird(obstaclePosition)
            ObstacleObject.connectToSwarmController(SwarmController1)

        # History Data
        AR1PositionHistory = np.array([[AR1.position[0], AR1.position[1], AR1.position[2]]])
        AR2PositionHistory = np.array([[AR2.position[0], AR2.position[1], AR2.position[2]]])
        AR3PositionHistory = np.array([[AR3.position[0], AR3.position[1], AR3.position[2]]])

        VSPositionHistory = np.array([VS.FRPPosition])

        responseValue = 0

        for iteration in range (10000):
            if iteration%5 == 0:
                currentRealPointPos = [AR1.position, AR2.position, AR3.position]
                VS.setRealPointPositions(currentRealPointPos)
                SwarmController1.calculateAgentsForces()
                APFForce = SwarmController1.agents[0].calculate_total_force()
                newTargetPos = VS.calculateNewVSPoint(APFForce)
                yawTarget1 = np.arctan2(newTargetPos[0][1], newTargetPos[0][0])
                yawTarget2 = np.arctan2(newTargetPos[1][1], newTargetPos[1][0])
                yawTarget3 = np.arctan2(newTargetPos[2][1], newTargetPos[2][0])

            AR1.controlPositionYaw((newTargetPos[0][0], newTargetPos[0][1], newTargetPos[0][2]), yawTarget1)
            AR2.controlPositionYaw((newTargetPos[1][0], newTargetPos[1][1], newTargetPos[1][2]), yawTarget2)
            AR3.controlPositionYaw((newTargetPos[2][0], newTargetPos[2][1], newTargetPos[2][2]), yawTarget3)
        
            AR1.updateState()
            AR2.updateState()
            AR3.updateState()

            if iteration%5 == 0:
                AR1PositionHistory = np.concatenate(
                    (AR1PositionHistory, np.array([[
                        AR1.position[0],
                        AR1.position[1],
                        AR1.position[2],
                    ]]))
                    )
                AR2PositionHistory = np.concatenate(
                    (AR2PositionHistory, np.array([[
                        AR2.position[0],
                        AR2.position[1],
                        AR2.position[2],
                    ]]))
                    )
                AR3PositionHistory = np.concatenate(
                    (AR3PositionHistory, np.array([[
                        AR3.position[0],
                        AR3.position[1],
                        AR3.position[2],
                    ]]))
                    )
                VSPositionHistory = np.concatenate(
                    (VSPositionHistory, np.array([
                        VS.FRPPosition
                    ]))
                )
                # if collide return False
                try:
                    for obstaclePosition in ObstaclesPosition:
                        distanceWithObstacle = abs(np.array(VS.FRPPosition) - np.array(obstaclePosition))
                        # print(distanceWithObstacle)
                        # print(math.sqrt(distanceWithObstacle[0]**2 + distanceWithObstacle[1]**2 + distanceWithObstacle[2]**2))
                        if math.sqrt(distanceWithObstacle[0]**2 + distanceWithObstacle[1]**2 + distanceWithObstacle[2]**2) < 1:
                            print("NABRAKK")
                            raise Exception("NABRAKKK")
                except:
                        return VSPositionHistory, False
            # print(VS.FRPPosition[0], initialVSPos[0], VS.FRPPosition[1], initialVSPos[1])
            # responseValue = responseValue + (VS.FRPPosition[0] - initialVSPos[0] + VS.FRPPosition[1] - initialVSPos[1])
            # initialVSPos = VS.FRPPosition
        # for i in range(len(VSPositionHistory) - 1):
        #     responseValue = responseValue + (VSPositionHistory[i+1][0] - VSPositionHistory[i][0])**2 + (VSPositionHistory[i+1][1] - VSPositionHistory[i][1])**2

        # if abs(VS.FRPPosition[0] - 10) > 0.2 and abs(VS.FRPPosition[1] - 10) > 0.2:
        #     return VSPositionHistory, False
        return VSPositionHistory, True

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
                                 [48.49, 14.29, 0.0]]  # PID z dot

        positionControllerPID = [[323.35, 8.38, 177.91],  # PID x
                                [138.38, 126.43, 98.03],    # PID y
                                [4.31, 8.1,  8.44]]  # PID z

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
        for outputValue in outputValues:
            # print("pos VS", outputValue)
            error = np.append(error, 20.0 - outputValue[0] - outputValue[1])
        # print('TOTAL ERROR', np.sum(abs(error)))
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
            [systemResponse, isStable] = self.calculateVSOnly(
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
        lastPosition = ''
        while (not isConvergent):
            for particle_index in range(particles):
                # Count
                # print('BEFORE particle velocity', particle_velocity[particle_index])

                particle_velocity[particle_index] = w*particle_velocity[particle_index] + c1*random.random()*(
                    particle_position_best[particle_index]-particle_position[particle_index]) + c2*random.random()*(global_best_position-particle_position[particle_index])

                # print('AFTER particle velocity', particle_velocity[particle_index])

                particle_position[particle_index] = np.round(particle_velocity[particle_index] + \
                    particle_position[particle_index], 2)

                # For Swarm
                # [systemResponse, isStable] = self.calculateSwarmDrones(particle_position[particle_index])
                # For Attitude and Position Controller
                [systemResponse, isStable] = self.calculateVSOnly(
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
                    lastPosition = systemResponse[-1]

            # Animation
            # plt.pause(0.1)
            # animation_pos._offsets3d = (
            #     particle_position[:, 0], particle_position[:, 1], particle_position[:, 2])
            # plt.draw()

            # Calculate position average
            position_average = np.round([np.mean(particle_position[:][0]), np.mean(particle_position[:][1]), np.mean(particle_position[:][2]), np.mean(particle_position[:][3])],2)

            iterate = iterate+1
            print('---------------------- ITERATION ', iterate, ' ----------------------')
            print("Global best fitness: ", global_fit_value, ", Parameters :",  np.round(global_best_position, 2))
            print("Last Position", lastPosition)
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
        plt.ylabel("Output")

        # For APF Swarm 
        # plt.title("Distance between Drones")
        # [bestResponse, isStable] = self.calculateSwarmDrones(self.bestParameter)

        # For Attitude and Position Controller
        plt.title("Response and PID (from PSO parameter)")
        [bestResponse, isStable] = self.calculateQuadrotorResponse(
            self.bestParameter)
        plt.plot(self.simulationTime, bestResponse, label="Best Response")
        plt.plot(self.simulationTime, self.targetOutput *
                 np.ones(self.simulationTime.shape), label="Set Point", color="red")
        plt.legend(loc='upper left')
        plt.show()
        plt.pause(100)
