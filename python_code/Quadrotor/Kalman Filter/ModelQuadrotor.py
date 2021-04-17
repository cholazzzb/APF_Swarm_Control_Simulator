import numpy as np
from numba import njit

g = 9.81
mass = 0.445
Ix = 0.0027
Iy = 0.0029
Iz = 0.0053

initialStateX = np.array(
    [[0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0]])

A = np.array([[0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
              [0, -g, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
              [g, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]])

B = np.array([[0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 1/Ix, 0, 0],
              [0, 0, 1/Iy, 0],
              [0, 0, 0, 1/Iz],
              [0, 0, 0, 0],
              [0, 0, 0, 0],
              [1/mass, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0]])

C = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])

D = np.array([[0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0]])

class ModelQuadrotor(object):
    def __init__(self, dt):
        self.dt = dt
        self.stateX = initialStateX

    def calculateXDot(self, u):
        return np.add(np.dot(A, self.stateX), np.dot(B, u))

    def calculateY(self, u):
        return np.add(np.dot(C, self.stateX), np.dot(D, u))

    def calculateDynamics(self, u):
        dynamics = self.calculateY(u)
        print('DYNAMCIS', dynamics)
        self.stateX = np.add(self.stateX, self.dt*self.calculateXDot(u))
        return dynamics
