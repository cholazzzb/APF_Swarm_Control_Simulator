from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

class DynamicGraphSingle(object):
    def __init__(self, dt):
        plt.ion()
        self.figure = plt.figure()
        self.xAxis = [0]
        self.dt = dt
        self.yAxis = []
        self.smallestYValue = []
        self.biggestYValue = []
        self.axs = []
        self.plots = []

        size = 111
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
            self.axs[index].set_ylim(self.smallestYValue[index]-0.2, self.biggestYValue[index]+0.2)
        if newYData < self.smallestYValue[index]:
            self.smallestYValue[index] = newYData
            self.axs[index].set_ylim(self.smallestYValue[index]-0.2, self.biggestYValue[index]+0.2)

        self.axs[index].set_xlim(0, self.xAxis[-1]+self.dt)

    def updatePlot(self):
        self.xAxis.append(self.xAxis[-1]+self.dt)
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()

    def pause(self, duration):
        plt.pause(duration)

    
