from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

class DynamicGraph(object):
    def __init__(self, dt):
        plotSize = 3 # plotSize always 3 WKWKW
        plt.ion()
        self.figure = plt.figure()
        self.xAxis = [0]
        self.dt = dt
        self.yAxis = []
        self.smallestYValue = []
        self.biggestYValue = []
        self.axs = []
        self.plots = []
        for index in range(plotSize):
            size = plotSize*100 + 200 + 10 + (index+1)*2 - 1 
            newAx = self.figure.add_subplot(size)
            self.axs.append(newAx)

    def createPlot(self, index, title, initialYData, style):
        self.yAxis.append([initialYData])
        self.axs[index].set_title(title)
        newPlot, = self.axs[index].plot(self.xAxis, [initialYData], style)
        self.biggestYValue.append(initialYData)
        self.smallestYValue.append(initialYData)
        self.plots.append(newPlot)

    def updatePlotData(self, index, newYData):
        self.yAxis[index].append(newYData)
        self.plots[index].set_xdata(self.xAxis)
        self.plots[index].set_ydata(self.yAxis[index])
        self.axs[index].set_xlim(0, self.xAxis[-1]+self.dt)
        if newYData > self.biggestYValue[index]:    
            self.biggestYValue[index] = newYData    
            self.axs[index].set_ylim(self.smallestYValue[index], self.biggestYValue[index])
        if newYData < self.smallestYValue[index]:
            self.smallestYValue[index] = newYData
            self.axs[index].set_ylim(self.smallestYValue[index], self.biggestYValue[index])

        self.axs[index].set_xlim(0, self.xAxis[-1]+self.dt)

    def updatePlot(self):
        self.xAxis.append(self.xAxis[-1]+self.dt)
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()

    def pause(self, duration):
        plt.pause(duration)

    
