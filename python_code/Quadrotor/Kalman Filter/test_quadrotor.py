import numpy as np
from ModelQuadrotor import ModelQuadrotor
from DynamicGraph import DynamicGraph
from KalmanFilter import KalmanFilter

dt = 0.01
simulationTime = 5.0
Quadrotor = ModelQuadrotor(dt)
KF = KalmanFilter()
P = np.diag()


Graph = DynamicGraph(1)
EstimationGraph = DynamicGraph(1)
inputU = np.array([[1], [0], [0], [0]])

style = ['r-', 'g-', 'b-']
style2 = ['r*', 'g*', 'b*']

response = Quadrotor.calculateDynamics(inputU)
Graph.createPlot(0, 'x Pos', response[0][0], style[0])
Graph.createPlot(1, 'y Pos', response[1][0], style[1])
Graph.createPlot(2, 'z Pos', response[2][0], style[2])

for i in range(int(simulationTime/dt)):
    # KALMAN PREDICTION
    (Quadrotor.stateX, )


    # TRUE
    response = Quadrotor.calculateDynamics(inputU)
    Graph.updatePlotData(0, response[0][0])
    Graph.updatePlotData(1, response[1][0])
    Graph.updatePlotData(2, response[2][0])

    Graph.updatePlot()

Graph.pause(20)
