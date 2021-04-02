import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

data = [[10, 10, 0.84496124031007758],
        [10, 20, 0.87209302325581395],
        [10, 30, 0.88139534883720927],
        [20, 10, 0.86201550387596892],
        [20, 20, 0.87441860465116272],
        [20, 30, 0.88992248062015500],
        [30, 10, 0.87984496124031009],
        [30, 20, 0.89922480620155043],
        [30, 30, 0.92015503875968996]]

x, y, z = zip(*data)

print('x', x, type(x))
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x, y, z)
plt.show()
