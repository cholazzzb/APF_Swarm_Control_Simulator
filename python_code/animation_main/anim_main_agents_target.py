from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

from DotAgent import DotAgent
import sys
sys.path.append('../')
from Agent import Agent
from SwarmPotentialField import SwarmPotentialField
from Target import Target
from TargetPotentialField import TargetPotentialField

min_allowable_dist = 1

Drones = []
position_drone1 = (4, 0, 5)
Drone1 = Agent(0, position_drone1, 1.5)
Drones.append(Drone1)
DroneAnim1 = DotAgent(position_drone1)

position_drone2 = (6, 5, 5)
Drone2 = Agent(1, position_drone2, 1)
Drones.append(Drone2)
DroneAnim2 = DotAgent(position_drone2)

# position_drone3 = (0, 13, 9)
# Drone3 = Agent(2, position_drone3, 1)
# Drones.append(Drone3)
# DroneAnim3 = DotAgent(position_drone3)

target1_position = (4,16,5) 
target2_position = (6,21,5)
Ship = Target(target1_position)
Ship2 = Target(target2_position)
Ships = [Ship, Ship2]

SPF = SwarmPotentialField(min_allowable_dist)
bestParameter = [0.48668045, 0.06204389, 0.76718196, 0.30186708]
SPF.setup(bestParameter)

# SPF.setPositiveGain1(1)
# SPF.setDampingFactor(0.5)
# SPF.setPositiveGain3(0.3)
TPF = TargetPotentialField(1,1,20)

plt.ion()
# Report
report = plt.figure()
ax1 = report.add_subplot(111)
ax1.set_title("Distance between quadrotor 1 and 2")
# ax2 = report.add_subplot(513)
# ax2.set_title("Distance between quadrotor 2 and 3")
# ax3 = report.add_subplot(515)
# ax3.set_title("Distance between quadrotor 3 and 1")

x_time = []
y_distance1 = []
max_distance1 = 0
y_distance2 = []
max_distance2 = 0
y_distance3 = []
max_distance3 = 0


dis1, = ax1.plot(x_time, y_distance1, 'r-')
# dis2, = ax2.plot(x_time, y_distance2, 'g-')
# dis3, = ax3.plot(x_time, y_distance3, 'b-')

# Animation
view = plt.figure()
arena = view.add_subplot(111, projection='3d')

arena.set_xlabel('X (meter)')
arena.set_ylabel('Y (meter)')
arena.set_zlabel('Z (meter)')

arena.set_xlim(-1, 21)
arena.set_ylim(-1, 21)
arena.set_zlim(0, 7)

drone1Position = arena.scatter([position_drone1[0]], [position_drone1[1]], [position_drone1[2]], color="red")
drone2Position = arena.scatter([position_drone2[0]], [position_drone2[1]], [position_drone2[2]], color="green")
# drone3Position = arena.scatter([position_drone3[0]], [position_drone3[1]], [position_drone3[2]], color="blue")

ship1Position = arena.scatter([target1_position[0]], [target1_position[1]], [target1_position[2]], color="black")
ship2Position = arena.scatter([target2_position[0]], [target2_position[1]], [target2_position[2]], color="black")

view.show()

for i in range(100):
    print('------- ITERATION ', i, '-----------')

    Drone1.SwarmPotentialForce = SPF.calculate_total_swarm_field_force(Drone1.index, Drones)
    print('Drone1 SPF = ', Drone1.SwarmPotentialForce, '\n')
    Drone2.SwarmPotentialForce = SPF.calculate_total_swarm_field_force(Drone2.index, Drones)
    print('Drone2 SPF = ', Drone2.SwarmPotentialForce, '\n')
    # Drone3.SwarmPotentialForce = SPF.calculate_total_swarm_field_force(Drone3.index, Drones)
    # print('Drone3 SPF = ', Drone3.SwarmPotentialForce, '\n')

    Drone1.TargetPotentialForce = TPF.calculate_target_force(Drone1.index, 0, Drones, Ships)
    Drone2.TargetPotentialForce = TPF.calculate_target_force(Drone2.index, 1, Drones, Ships)

    Drone1.calculateVelocity(Drone1.calculate_total_force())
    print('Drone1 Velocity', Drone1.velocity)
    Drone1.move()
    print('Drone1 Position', Drone1.position)

    Drone2.calculateVelocity(Drone2.calculate_total_force())
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
    # y_distance2.append(newDis2)
    # dis2.set_ydata(y_distance2)
    # dis2.set_xdata(x_time)
    # ax2.set_xlim(0, i)
    # if newDis2 > max_distance2:
    #     max_distance2 = newDis2
    #     ax2.set_ylim(0, max_distance2)

    # newDis3 = SPF.getDistance(Drone3.index, Drone1.index, Drones)[1]
    # y_distance3.append(newDis3)
    # dis3.set_ydata(y_distance3)
    # dis3.set_xdata(x_time)
    # ax3.set_xlim(0, i)
    # if newDis3 > max_distance3:
    #     max_distance3 = newDis3
    #     ax3.set_ylim(0, max_distance3)

    report.canvas.draw()
    report.canvas.flush_events()

    print('\n')
