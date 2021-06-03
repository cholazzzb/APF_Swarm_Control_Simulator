import numpy as np
from numpy.random import randn
from Model import Model
from DynamicGraph import DynamicGraph
from KalmanFilter import KalmanFilter
import matplotlib.pyplot as plt

A = np.array([[-3, -2, -1], [1, 0, 0], [0, 1, 0]])
B = np.array([[1], [0], [0]])
C = np.array([[0, 0, 1]])
D = np.array([[0]])
X = np.array([[0.0], [0.0], [0.0]])

dt = 0.01
simulationTime = 5.0
System = Model([A, B, C, D], X, dt)
KF = KalmanFilter()

# Graph = DynamicGraph(1)
inputU = np.array([[1]])

style = ['r-', 'g-', 'b-']

response = System.calculateDynamics(inputU)
# Graph.createPlot(0, 'Response Y', response[0][0], style[0])
# Graph.createPlot(1, 'Kalman', 0, style[1])

# Kalman Filter setup
P = np.diag((0.1, 0.1, 0.1))
Q = np.eye(X.shape[0])

comparePlotTrue = []
comparePlotKalman0 = []
comparePlotKalman1 = []
comparePlotKalman2 = []

for i in range(int(simulationTime/dt)):
    response = System.calculateDynamics(inputU)
    comparePlotTrue.append(response[0][0])
    ## Measurement matrices 
    # print('before ', response)
    Y = response + abs(randn(1)[0]/100)
    H = np.array([[1, 0, 0], [0, 1, 0]])
    R = np.eye(Y.shape[0])

    # Kalman Algorithm
    (X, P) = KF.predict(X, P, A, Q, B, inputU)
    (X, P, K, Ktmin, CPkCt, LH) = KF.update(X, P, Y, H, R)
    comparePlotKalman0.append(X[0][0])
    comparePlotKalman1.append(X[1][0])
    comparePlotKalman2.append(X[2][0])

    # print('after ', Y)
    # Graph.updatePlotData(0, response[0][0])
    # Graph.updatePlotData(1, X[0][0])
    # Graph.updatePlot()

plt.plot(np.linspace(0, 5, 500), comparePlotTrue, label="True Val")
plt.plot(np.linspace(0, 5, 500), comparePlotKalman0, label="Kalman0")
plt.plot(np.linspace(0, 5, 500), comparePlotKalman1, label="Kalman1")
# plt.plot(np.linspace(0, 5, 500), comparePlotKalman2, label="Kalman2")
plt.legend(loc='lower right')

plt.show()

# Graph.pause(20)