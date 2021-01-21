from tupleUtil import *

class Target(object):
    def __init__(self, position):
        self.position = position
        self.positionHistory = [position]
        self.velocity = (0,0,0)

    def setVelocity(self, newVelocity):
        self.velocity = newVelocity

    def getVelocity(self):
        return self.velocity

    def move(self):
        self.position = plusWithTuple(self.position, self.velocity)