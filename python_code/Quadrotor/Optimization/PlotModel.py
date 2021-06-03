import matplotlib.pyplot as plt
import numpy as np
import math

import sys
sys.path.append('../')
from Quadrotor import Quadrotor
radianToDegree = 180/math.pi
degreeToRadian = math.pi/180

# PID Simulation
dt = 0.33
totalData = 18
simulationTime = []

for iteration in range(totalData):
    simulationTime.append(dt*iteration)
simulationTimeNumpy = np.array(simulationTime)

specs = {"mass": 0.445, "inertia": [
    0.0027, 0.0029, 0.0053], "armLength": 0.125}
initialState = [[0.0, 0.0, 1.0], [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
initialInput = [0.445 * 9.8, 0.0, 0.0, 0.0]
attitudeControllerPID = [[1.43, 0, 0.13],  # PID phi
                         [1.52, 0, 0.14],  # PID theta
                         [2.43, 0, 0.26],  # PID psi
                         [48.49, 14.29, 0]]  # PID z dot

positionControllerPID = [[0, 0, 0],  # PID x
                         [0, 0, 0],    # PID y
                         [0, 0,  0]]  # PID z

model = Quadrotor(0, "model", specs, initialState, initialInput, attitudeControllerPID, positionControllerPID)

responseValue = []
for iteration in range(totalData):
    model.controlAttitude([-9, 0, 0, 0])
    model.updateState()
    responseValue.append(model.angles[0]*radianToDegree)

# Real Data
realOutput = [-0.35039999999999993, -0.298, -0.26280000000000003, -0.225, -0.194,
              -0.1666, -0.1666, -0.149, -0.097, -0.1558, 0.6363999999999999, 0.6706,
              0.8261999999999998, 0.14180000000000006, -0.28500000000000003, -0.6734,
              -7.5488, -9.0636]

plt.plot(simulationTime, realOutput, label="realOutput")
plt.plot(simulationTime, responseValue, label="Modelling")
plt.legend(loc="lower left")
plt.title("Phi Angle")
plt.show()
