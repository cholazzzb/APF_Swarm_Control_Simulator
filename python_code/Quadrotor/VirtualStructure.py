import numpy as np
import math
from copy import deepcopy

radianToDegree = 180/math.pi
degreeToRadian = math.pi/180

import sys
sys.path.append('../../')
from Agent import Agent

'''
--- shapeList ---
return [(FRPPosition), VSPoint1, ...)]
'''
shapeList = {
    "triangle": [[0, 1, 0], [0.5, -0.5, 0], [-0.5, -0.5, 0]],
    "square": [[], [], [], []]
}

class VirtualStructure(object):
    def __init__(self, shape, heading):
        self.FRPPosition = [0,0,0] # FRP = Formation Reference Point
        self.heading = heading # radian [phi, theta, psi]
        self.shape = shape
        # VRP = Virtual Structure Point
        self.VSPPositions = self.calculateVSPoint(self.FRPPosition)
        
        self.maxSpeed = 5
        self.mass = 0.445 # AR Drone Mass

        # Real Position
        self.realPointPositions = []

    '''
    --- calculateVSPoint ---
    transform VRP position into position in global axis.
    return newVRPPositions
    '''
    def calculateVSPoint(self, FRPPosition):
        [phi, theta, psi] = self.heading
        ## Only use psi
        transformationMatrix = [
            [math.cos(psi), math.sin(psi), 0], #x
            [-math.sin(psi), math.cos(psi), 0], #y
            [0, 0, 1] #z (z is point to up)
        ]
        newVRPPositions = []
        for shapePoint in shapeList.get(self.shape):
            newVRPPositions.append(FRPPosition + np.dot(transformationMatrix, shapePoint))
        return newVRPPositions

    def setFRPPosition(self, newFRPPosition):
        self.FRPPosition = newFRPPosition
        self.VSPPositions = self.calculateVSPoint(self.FRPPosition)

        # Set VS Point altitude
        for VRPPPosition in self.VSPPositions:
            VRPPPosition[2] = newFRPPosition[2]

    def setRealPointPositions(self, newRealPointPositions):
        self.realPointPositions = newRealPointPositions
    
    def calculateMoveRange(self):
        # dt = 0.1
        return 0.1 * self.maxSpeed

    def calculateFRPVel(self, APFForce):
        # print('APFForce', APFForce)
        FRPVel = [0,0,0]
        FRPVel[0] = APFForce[0]/self.mass
        FRPVel[1] = APFForce[1]/self.mass
        FRPVel[2] = APFForce[2]/self.mass

        return FRPVel

    # Calculate each VRP Position
    def moveFRP(self, APFForce):
        velocity = self.calculateFRPVel(APFForce)
        newFRPPosition = deepcopy(self.FRPPosition)
        newFRPPosition[0] = self.FRPPosition[0] + velocity[0]
        newFRPPosition[1] = self.FRPPosition[1] + velocity[1]
        # newFRPPosition[2] = self.FRPPosition[2] + velocity[2]
        newFRPPosition[2] = 5
       
        newVSPositions = self.calculateVSPoint(newFRPPosition)
        VSsVelocities = [[0,0,0], [0,0,0], [0,0,0]] # For Triangle

        for index in range(len(VSsVelocities)):
            # print('VRP New - current', newVSPositions[index][0], self.VSPPositions[index][0])
            VSsVelocities[index][0] = newVSPositions[index][0] - self.VSPPositions[index][0]
            VSsVelocities[index][1] = newVSPositions[index][1] - self.VSPPositions[index][1]
            VSsVelocities[index][2] = newVSPositions[index][2] - self.VSPPositions[index][2]
        moveRange = self.calculateMoveRange()

        # print('VsVelocities before', VSsVelocities)
        for VsVelocities in VSsVelocities:
            for velIndex in range(len(VsVelocities)):
                if VsVelocities[velIndex] > moveRange:
                    velocity[velIndex] = moveRange
                if VsVelocities[velIndex] < -moveRange:
                    velocity[velIndex] = -moveRange
        # print('VsVelocities after', velocity)
        
        newFRPPosition = self.FRPPosition
        newFRPPosition[0] = self.FRPPosition[0] + velocity[0]
        newFRPPosition[1] = self.FRPPosition[1] + velocity[1]
        # newFRPPosition[2] = self.FRPPosition[2] + velocity[2]
        newFRPPosition[2] = 5

        self.setFRPPosition(newFRPPosition)
        self.SwarmController.agents[0].position = (newFRPPosition[0], newFRPPosition[1], newFRPPosition[2])

    def calculateNewVSPoint(self, APFForce):
        quadrotorInVP = 0
        for agentIndex in range(len(self.realPointPositions)):
            for agentPos in range(len(self.realPointPositions[agentIndex])):
                # print("distance real - virtual", round(abs(self.realPointPositions[agentIndex][agentPos] - self.VSPPositions[agentIndex][agentPos])*100)/100)
                if abs(self.realPointPositions[agentIndex][agentPos] - self.VSPPositions[agentIndex][agentPos]) < 0.01 :
                    quadrotorInVP = quadrotorInVP + 1 
        if quadrotorInVP == 9: # Quadrotor * 3 pos
            # print("inVP")
            self.moveFRP(APFForce)

        return self.VSPPositions

    def connectToSwarmController(self, SwarmController):
        [x, y, z] = self.FRPPosition
        swarmAgent = Agent(0, (x,y,z), self.mass)
        SwarmController.addAgent(swarmAgent)
        self.SwarmController = SwarmController