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


class FlyHistory(object):
    # xlim, ylim, zlim = array -> [lower, upper]
    def __init__(self, xlim, ylim, zlim):
        fig = plt.figure(figsize=(15, 15))
        self.ax = fig.add_subplot(111, projection='3d')

        self.ax.set_xlabel('X Pos')
        self.ax.set_ylabel('Y Pos')
        self.ax.set_zlabel('X Pos')

        self.ax.set_xlim(xlim[0], xlim[1])
        self.ax.set_ylim(ylim[0], ylim[1])
        self.ax.set_zlim(zlim[0], zlim[1])

        plt.title('Fly History')

    # xPos, yPos, zPos = array
    # size = integer -> diameter object in (m)
    # color = string -> "red" or "g"
    def addObject(self, xPos, yPos, zPos, size, color, opacity):
        self.ax.plot(xPos.pop(), yPos.pop(), zPos.pop(), 'o',
                     markersize=size*100, color='black', alpha=1)
        self.ax.plot(xPos, yPos, zPos, 'o', markersize=size*100,
                     color=color, alpha=opacity)

    def draw(self):
        plt.draw()
        plt.show()
