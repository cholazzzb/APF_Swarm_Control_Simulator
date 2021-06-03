import numpy as np

class Model(object):
    def __init__(self, stateSpace, initialStateX, dt):
        self.dt = dt
        self.stateX = initialStateX
        self.A = stateSpace[0]
        self.B = stateSpace[1]
        self.C = stateSpace[2]
        self.D = stateSpace[3]

    def calculateXDot(self, u):
        return np.add(np.dot(self.A, self.stateX), np.dot(self.B, u))

    def calculateY(self, u):
        return np.add(np.dot(self.C, self.stateX), np.dot(self.D, u))

    def calculateDynamics(self, u):
        dynamics = self.calculateY(u)
        # print('DYNAMCIS', dynamics)
        print('statex', self.stateX[2])
        print('Y', dynamics)
        self.stateX = np.add(self.stateX, self.dt*self.calculateXDot(u))
        return dynamics
