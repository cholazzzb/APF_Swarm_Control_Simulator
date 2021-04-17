from tupleUtil import *


class Agent(object):
    def __init__(self, index, position, mass):
        self.index = index
        self.targetIndex = -1
        self.position = position
        self.positionHistory = [position]
        self.mass = mass
        self.velocity = (0, 0, 0)
        self.maxVelocity = 0.3 # absolute value, need to check tello spec
        self.ObstaclePotentialForce = (0, 0, 0)
        self.SwarmPotentialForce = []
        self.TargetPotentialForce = (0, 0, 0)
        self.totalForce = (0, 0, 0)
        self.targetIndex = 0

    def calculate_total_force(self):
        return plusWithTuple(self.SwarmPotentialForce, plusWithTuple(self.ObstaclePotentialForce, self.TargetPotentialForce)) 

    def setVelocity(self, newVelocity):
        for index in range(0, len(newVelocity)):
            xVel, yVel, zVel = newVelocity
            if xVel > self.maxVelocity:
                xVel = self.maxVelocity
            elif xVel < -self.maxVelocity:
                xVel = -self.maxVelocity
            if yVel > self.maxVelocity:
                yVel = self.maxVelocity
            elif yVel < -self.maxVelocity:
                yVel = -self.maxVelocity
            if zVel > self.maxVelocity:
                zVel = self.maxVelocity
            elif zVel < -self.maxVelocity:
                zVel = -self.maxVelocity
            newVelocity = (xVel, yVel, zVel)

        self.velocity = newVelocity

    def getVelocity(self):
        return self.velocity

    def calculateVelocity(self, newTotalForce):
        deltaForce = minusWithTuple(newTotalForce, self.totalForce)
        velocityGain = divideWithInteger(deltaForce, self.mass)
        newVelocity = plusWithTuple(self.velocity, velocityGain)
        self.setVelocity(newVelocity)

    def move(self):
        # print('vel', self.velocity)
        self.position = plusWithTuple(self.position, self.velocity)
        self.positionHistory.append(self.position)

    def calculate_obstacles_potential_forces(self, Obstacles):
        total_obstacles_potential_forces = (0, 0, 0)
        for Obstacle in Obstacles:
            total_obstacles_potential_forces = plusWithTuple(
                total_obstacles_potential_forces, Obstacle.ObstaclePotentialForces)
            # If there is no obstacle detected, the potential force return to 0
            self.ObstaclePotentialForce = (0, 0, 0)

    def updatePosition(self, newPosition):
        self.position = newPosition
