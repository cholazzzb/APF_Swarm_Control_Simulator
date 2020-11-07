from tupleUtil import *

class Agent(object):
    def __init__(self, position, mass):
        self.position = position
        self.mass = mass
        self.velocity = (0, 0, 0)
        self.TargetPotentialForce = (0, 0, 0)
        self.totalForce = (0, 0, 0)

    def setVelocity(self, newVelocity):
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
