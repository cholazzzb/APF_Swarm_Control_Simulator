import numpy as np

class DotAgent(object):
    def __init__(self, initialPosition):
        self.position = np.array(initialPosition)

    def getBodyPosition(self):
        return np.array([self.position, self.position], dtype="object")