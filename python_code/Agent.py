from tupleUtil import *


class Agent(object):
    def __init__(self, index, position, mass):
        self.index = index
        self.targetIndex = -1
        self.position = position
        self.positionHistory = [position]
        self.mass = mass
        self.velocity = (0, 0, 0)
        self.maxVelocity = 5 # absolute value
        self.ObstaclePotentialForce = (0, 0, 0)
        self.SwarmPotentialForce = []
        self.TargetPotentialForce = (0, 0, 0)
        self.totalForce = (0, 0, 0)
        self.targetIndex = 0

    def calculate_total_force(self):
        return plusWithTuple(self.ObstaclePotentialForce, self.TargetPotentialForce)

    def setVelocity(self, newVelocity):
        for index in range(0, len(newVelocity)):
            xVel, yVel, zVel = newVelocity
            if abs(xVel) > self.maxVelocity:
                xVel = self.maxVelocity
                newVelocity = (xVel, yVel, zVel)
            if abs(yVel) > self.maxVelocity:
                yVel = self.maxVelocity
                newVelocity = (xVel, yVel, zVel)
            if abs(zVel) > self.maxVelocity:
                zVel = self.maxVelocity
                newVelocity = (xVel, yVel, zVel)

        self.velocity = newVelocity

    def getVelocity(self):
        return self.velocity

    def calculateVelocity(self, newTotalForce):
        deltaForce = minusWithTuple(newTotalForce, self.totalForce)
        velocityGain = divideWithInteger(deltaForce, self.mass)
        newVelocity = plusWithTuple(self.velocity, velocityGain)
        self.setVelocity(newVelocity)
        return '--set new velocity success--'

    def move(self):
        self.position = plusWithTuple(self.position, self.velocity)
        self.positionHistory.append(self.position)

    def calculate_obstacles_potential_forces(self, Obstacles):
        total_obstacles_potential_forces = (0, 0, 0)
        for Obstacle in Obstacles:
            total_obstacles_potential_forces = plusWithTuple(
                total_obstacles_potential_forces, Obstacle.ObstaclePotentialForces)
            # If there is no obstacle detected, the potential force return to 0
            self.ObstaclePotentialForce = (0, 0, 0)
