import numpy as np
import matplotlib.pyplot as plt
import math

from DynamicGraph import DynamicGraph
import sys
sys.path.append('../')
from Quadrotor import Quadrotor
from Report import Report

# Setup
# Time
dt = 0.01
startTime = 0
endTime = 5

simulationTime = np.linspace(
    startTime, endTime, 1 + int((endTime-startTime)/dt))

# Output
# target 1, 2, 3...
# outputSetopint = [[x1,y1,z1], [x2,y2,z2], ...] in (m)
outputSetpoint = [[0, 1, 1], [5, 0, 1], [5, 5, 1], [0, 0, 1], [0, 0, 0]]

specs = {"mass": 0.445, "inertia": [
    0.0027, 0.0029, 0.0053], "armLength": 0.125}
initialState = [[0.0, 0.0, 1.0], [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
initialInput = [0.0, 0.0, 0.0, 0.0]
attitudeControllerPID = [[1.43, 0.0, 0.13],  # PID phi
                         [1.52,  0.0, 0.14],  # PID theta
                         [2.43, 0, 0.26],  # PID psi
                         [48.49, 14.29, 0.0]]  # PID z dot

positionControllerPID = [[79.58, 26.94, 38.05],  # PID x
                         [122.15, 96.66, 4.61 ],  # PID y
                         [4.31, 8.1, 8.44]]  # PID z

model = Quadrotor(0, "AR1", specs, initialState,
                       initialInput, attitudeControllerPID, positionControllerPID)

Report1 = Report(model)
targetAttitude = DynamicGraph(0.01)
targetAttitude.createPlot(0, "X", 0, 'g-')
targetAttitude.createPlot(1, "Theta", 0, "r-")
targetAttitude.createPlot(2, "x err", 0, "b-")

for iteration in range(500):
    attitudeTarget = model.controlPosition([1,0,1])
    model.updateState()
    # print("angles", model.angles)
    # print("pos", model.position)
    Report1.updateReport(model.getState(), model.thrust, model.moments)
    targetAttitude.updatePlotData(0, model.position[0])
    targetAttitude.updatePlotData(1, attitudeTarget[6])
    targetAttitude.updatePlotData(2, attitudeTarget[7])
    targetAttitude.updatePlot()

Report1.generateReport()
plt.pause(200)
