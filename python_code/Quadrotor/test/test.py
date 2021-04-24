import matplotlib.pyplot as plt
import numpy as np

fig1 = plt.figure()
ax1 = fig1.add_subplot(111)

# ax1.plot(np.array([[1,2,3], [2,3,4]]), label=1, color="r")
# ax1.plot(np.array([[1,2,3], [3,3,3]]), label=2, color="g")
ax1.plot([0,1,2,3], [2,2,3,4], label=3, color="r")
ax1.plot([0,1,2,3], [2,2,2,2], label=3, color="g")
ax1.plot([0,1,2,3], [3,3,3,3], label=3, color="b")  

ax1.legend(loc=2)

plt.show()