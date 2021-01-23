from matplotlib import pyplot as plt

f=plt.figure()
ax=f.add_subplot(111, projection='3d')
mylines=ax.plot(xs,ys,zs)

#update figure with new x and z data
mylines[0].set_xdata(newxs)
mylines[0].set_3d_properties(newzs)
f.canvas.draw()