import sys
sys.path.append('../')

from DynamicGraph import DynamicGraph

Forces = DynamicGraph(0.01)
Forces.createPlot(0, 'Force 1', 1, 'g-')
Forces.createPlot(1, 'Force 2', 2, 'r-')
Forces.createPlot(2, 'Force 3', 3, 'b-')

for iteration in range (100):
    Forces.updatePlotData(0, iteration+1)
    Forces.updatePlotData(1, 2-iteration)
    Forces.updatePlotData(2, iteration*3)
    Forces.updatePlot()