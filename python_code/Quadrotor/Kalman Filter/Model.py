import numpy as np
from numba import njit

A = np.array([[-3, -2, -1], [1, 0, 0], [0, 1, 0]])
B = np.array([[1], [0], [0]])
C = np.array([[0, 0, 1]])
D = np.array([[0]])

class Model(object):
    def __init__(self,dt):
        self.dt = dt
        self.stateX = np.array([[0], [0], [0]])

    def calculateXDot(self, u):
        return np.add(np.dot(A, self.stateX), np.dot(B, u))

    def calculateY(self, u):
        return np.add(np.dot(C, self.stateX), np.dot(D, u))

    def calculateDynamics(self, u):
        dynamics = self.calculateY(u)
        print('DYNAMCIS', dynamics)
        self.stateX = np.add(self.stateX, self.dt*self.calculateXDot(u))
        return dynamics
