import numpy as np

import sys
sys.path.append('../')
from Target import Target


class Ship(object):
    def __init__(self, initialPosition, length):
        self.position = np.array(initialPosition)
        self.length = length

    def move(self, velocity):
        self.position = self.position + velocity

    def getBodyPosition(self):
        # Example front is +x
        frontPart = [self.position[0] + self.length,
                     self.position[1], self.position[2]]
        bodyPart = [[self.position[0], self.position[0], self.position[0] - self.length, self.position[0]-self.length],
                    [self.position[1] + self.length/2, self.position[1]-self.length/2, self.position[1] + self.length/2, self.position[1] - self.length/2],
                    [self.position[2], self.position[2], self.position[2], self.position[2]]]

        return np.array([frontPart, bodyPart], dtype="object")

    def connectToSwarmController(self, SwarmController):
        self.targetAgent = Target(
            (self.position[0], self.position[1], self.position[2]))
        SwarmController.addTarget(self.targetAgent)
