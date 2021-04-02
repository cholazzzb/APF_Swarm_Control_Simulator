from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

from DotAgent import DotAgent
import sys
sys.path.append('../')
from Agent import Agent
from SwarmPotentialField import SwarmPotentialField

min_allowable_dist = 1

Drones = []
position_drone1 = (0, 0, 5)
Drone1 = Agent(0, position_drone1, 1)
Drones.append(Drone1)
DroneAnim1 = DotAgent(position_drone1)

position_drone2 = (10, 0, 5)
Drone2 = Agent(1, position_drone2, 1)
Drones.append(Drone2)
DroneAnim2 = DotAgent(position_drone2)

# position_drone3 = (0, 13, 9)
# Drone3 = Agent(2, position_drone3, 1)
# Drones.append(Drone3)
# DroneAnim3 = DotAgent(position_drone3)

SPF = SwarmPotentialField(min_allowable_dist)
SPF.setup([0.3, 0.21, 0.56, 0.03])
# SPF.setPositiveGain1(1)
# SPF.setDampingFactor(0.5)
# SPF.setPositiveGain3(0.3)

plt.ion()
# Report
report = plt.figure()
ax1 = report.add_subplot(511)
ax1.set_title("Distance between quadrotor 1 and 2")
ax2 = report.add_subplot(513)
ax2.set_title("X Pos Drone 1")
ax3 = report.add_subplot(515)
ax3.set_title("X Pos Drone 2")

x_time = []
y_distance1 = []
max_distance1 = 0

y_distance2 = []
max_distance2 = 0
y_distance3 = []
max_distance3 = 0

x_pos1 = []
x_pos1_max = 0
x_pos2 = []
x_pos2_max = 0

dis1, = ax1.plot(x_time, y_distance1, 'r-')
dis2, = ax2.plot(x_time, y_distance2, 'g-')
dis3, = ax3.plot(x_time, y_distance3, 'b-')

# Animation
view = plt.figure()
arena = view.add_subplot(111, projection='3d')

arena.set_xlabel('X (meter)')
arena.set_ylabel('Y (meter)')
arena.set_zlabel('Z (meter)')

arena.set_xlim(-1, 12)
arena.set_ylim(-1, 12)
arena.set_zlim(6, 12)

drone1Position = arena.scatter([position_drone1[0]], [position_drone1[1]], [position_drone1[2]], color="red")
drone2Position = arena.scatter([position_drone2[0]], [position_drone2[1]], [position_drone2[2]], color="green")
# drone3Position = arena.scatter([position_drone3[0]], [position_drone3[1]], [position_drone3[2]], color="blue")

view.show()

for i in range(100):
    print('------- ITERATION ', i, '-----------')

    Drone1.SwarmPotentialForce = SPF.calculate_total_swarm_field_force(Drone1.index, Drones)
    print('Drone1 SPF = ', Drone1.SwarmPotentialForce, '\n')
    Drone2.SwarmPotentialForce = SPF.calculate_total_swarm_field_force(Drone2.index, Drones)
    print('Drone2 SPF = ', Drone2.SwarmPotentialForce, '\n')
    # Drone3.SwarmPotentialForce = SPF.calculate_total_swarm_field_force(Drone3.index, Drones)
    # print('Drone3 SPF = ', Drone3.SwarmPotentialForce, '\n')

    Drone1.calculateVelocity(Drone1.SwarmPotentialForce)
    print('Drone1 Velocity', Drone1.velocity)
    Drone1.move()
    print('Drone1 Position', Drone1.position)

    Drone2.calculateVelocity(Drone2.SwarmPotentialForce)
    print('Drone2 Velocity', Drone2.velocity)
    Drone2.move()
    print('Drone2 Position', Drone2.position)

    # Drone3.calculateVelocity(Drone3.SwarmPotentialForce)
    # print('Drone3 Velocity', Drone3.velocity)
    # Drone3.move()
    # print('Drone3 Position', Drone3.position)

    plt.pause(1)
    drone1Position._offsets3d = ([Drone1.position[0]], [Drone1.position[1]], [Drone1.position[2]])
    drone2Position._offsets3d = ([Drone2.position[0]], [Drone2.position[1]], [Drone2.position[2]])
    # drone3Position._offsets3d = ([Drone3.position[0]], [Drone3.position[1]], [Drone3.position[2]])
    plt.draw()

    x_time.append(i)

    newDis1 = SPF.getDistance(Drone1.index, Drone2.index, Drones)[1]
    y_distance1.append(newDis1)
    dis1.set_ydata(y_distance1)
    dis1.set_xdata(x_time)
    ax1.set_xlim(0, i)
    if newDis1 > max_distance1:
        max_distance1 = newDis1
        ax1.set_ylim(0, max_distance1)

    # newDis2 = SPF.getDistance(Drone2.index, Drone3.index, Drones)[1]
    x_pos1.append(Drone1.position[0])
    dis2.set_ydata(x_pos1)
    dis2.set_xdata(x_time)
    ax2.set_xlim(0, i)
    if Drone1.position[0] > x_pos1_max:
        x_pos1_max = Drone1.position[0]
        ax2.set_ylim(0, x_pos1_max)

    x_pos2.append(Drone2.position[0])
    dis3.set_ydata(x_pos2)
    dis3.set_xdata(x_time)
    ax3.set_xlim(0, i)
    if Drone2.position[0] > x_pos2_max:
        x_pos2_max = Drone2.position[0]
        ax3.set_ylim(0, x_pos2_max)

    report.canvas.draw()
    report.canvas.flush_events()

plt.pause(20)
