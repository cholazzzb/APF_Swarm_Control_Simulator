import matplotlib.pyplot as plt
import numpy as np

x = np.linspace(0, 6*np.pi, 100)
y = np.sin(x)

# You probably won't need this if you're embedding things in a tkinter plot...
plt.ion()

fig = plt.figure()
ax = fig.add_subplot(211)
line1, = ax.plot(x, y, 'r-')  # Returns a tuple of line objects, thus the comma
ax2 = fig.add_subplot(212)
line2, = ax.plot(x, y, 'b-')
line3, = ax2.plot(x,y, 'y-')

for phase in np.linspace(0, 10*np.pi, 500):
    line1.set_ydata(np.sin(x + phase))
    line2.set_ydata(np.cos(x + phase))
    line3.set_ydata(np.tan(x + phase))
    fig.canvas.draw()
    fig.canvas.flush_events()
