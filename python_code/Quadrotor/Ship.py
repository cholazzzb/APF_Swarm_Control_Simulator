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
        return np.array([[self.position[0] + self.length, self.position[0] + self.length, self.position[0] - self.length, self.position[0] - self.length], [self.position[1] + self.length, self.position[1] - self.length, self.position[1] + self.length, self.position[1] - self.length], [self.position[2], self.position[2], self.position[2], self.position[2]]
                         ])

    def connectToSwarmController(self, SwarmController):
        self.swarmAgent = Target((self.position[0], self.position[1],self.position[2]))
        SwarmController.addTarget(self.swarmAgent)