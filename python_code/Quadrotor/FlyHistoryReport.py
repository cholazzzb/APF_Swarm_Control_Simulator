import numpy as np
from numpy import *
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d


class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0, 0), (0, 0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        FancyArrowPatch.draw(self, renderer)


class FlyHistoryReport(object):
    def __init__(self, agent, arena):
        self.agent = agent
        self.arena = arena
        self.objectPositions = []

    # xPos, yPos, zPos = number
    # size = integer -> diameter object in (m)
    # color = string -> "red" or "g"
    def addObject(self, xPos, yPos, zPos, size, color, opacity):
        plt.pause(0.01)
        self.arena.plot([xPos], [yPos], [zPos], 'o', markersize=size*100,
                     color=color, alpha=opacity)
        plt.draw()

    def addHistory(self, xPosArray, yPosArray, zPosArray, size, color, opacity):
        self.arena.plot(xPosArray, yPosArray, zPosArray, 'o', markersize=size*100, color=color, alpha=opacity)
        plt.draw()