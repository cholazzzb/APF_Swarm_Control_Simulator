import numpy as np

import sys
sys.path.append('../')
from Obstacle import Obstacle

class Bird(object):
    def __init__(self, initialPosition):
        self.position = np.array(initialPosition)
    
    def getBodyPosition(self):
        return np.array([self.position, self.position], dtype="object")

    def connectToSwarmController(self, SwarmController):
        self.ObstacleAgent = Obstacle((self.position[0],self.position[1], self.position[2]))
        SwarmController.addObstacle(self.ObstacleAgent)