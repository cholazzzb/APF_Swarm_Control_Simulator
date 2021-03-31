import numpy as np
from Model import Model
from DynamicGraph import DynamicGraph

dt = 0.01
simulationTime = 5.0
System = Model(dt)
Graph = DynamicGraph(1)
inputU = np.array([[1]])

style = ['r-', 'g-', 'b-']

response = System.calculateDynamics(inputU)
Graph.createPlot(0, 'Response Y', response[0][0], style[0])

for i in range(int(simulationTime/dt)):
    response = System.calculateDynamics(inputU)
    Graph.updatePlotData(0, response[0][0])
    Graph.updatePlot()

Graph.pause(20)