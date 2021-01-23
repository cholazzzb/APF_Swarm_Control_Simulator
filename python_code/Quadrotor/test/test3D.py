import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

X = np.array([[0,0,0], [0,1,0], [0,-1,0], [1,0,0], [-1,0,0]])
Y = np.random.rand(5, 3)*0.5

plt.ion()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.set_xlabel('X (meter)')
ax.set_ylabel('Y (meter)')
ax.set_zlabel('Z (meter)')

ax.set_xlim(-1,5)
ax.set_ylim(-1,5)
ax.set_zlim(-1,5)

drone1 = ax.scatter(X[:, 0], X[:, 1], X[:, 2], color='red')
drone1line = ax.plot3D([1,1],[2,2],[3,3],color = 'green')
drone2 = ax.scatter(Y[:, 0], Y[:, 1], Y[:, 2], color='blue')
fig.show()

for i in range(0, 20):
    plt.pause(1)

    Y = np.random.rand(5, 3)*0.5

    drone1._offsets3d = (X[:,0], Y[:,1], Y[:,2])
    drone2._offsets3d = (X[:,0], Y[:,1], X[:,2])
    plt.draw()