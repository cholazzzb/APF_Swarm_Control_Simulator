from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

from DotAgent import DotAgent
import sys
sys.path.append('../')
from Agent import Agent
from SwarmPotentialField import SwarmPotentialField

quadrotorPath = sys.path[0].replace("animation_main", "Quadrotor") 
sys.path.append(quadrotorPath)
print('path', sys.path)
from Quadrotor import Quadrotor

min_allowable_dist = 1

specs = {"mass": 0.445, "inertia": [
    0.0027, 0.0029, 0.0053], "armLength": 0.125}
initialInput = [0.0, 0.0, 0.0, 0.0]
attitudeControllerPID = [[1.43, 0, 0.13],  # PID phi
                            [1.52, 0, 0.14],  # PID theta
                            [2.43, 0, 0.26],  # PID psi
                            [48.49, 14.29, 0.0]]  # PID z dot

positionControllerPID = [[323.35, 8.38, 177.91],  # PID x
                        [138.38, 126.43, 98.03],    # PID y
                        [4.31, 8.1,  8.44]]  # PID z


Drones = []
position_drone1 = (0, 0, 5)
Drone1 = Agent(0, position_drone1, 1)
Drones.append(Drone1)
DroneAnim1 = DotAgent(position_drone1)
initialState1 = [[0.0, 0.0, 5.0], [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]

AR1 = Quadrotor(0, "AR1", specs, initialState1, initialInput, attitudeControllerPID, positionControllerPID)

position_drone2 = (10, 0, 5)
Drone2 = Agent(1, position_drone2, 1)
Drones.append(Drone2)
DroneAnim2 = DotAgent(position_drone2)
initialState2 = [[10.0, 0.0, 5.0], [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]

AR2 = Quadrotor(0, "AR2", specs, initialState2, initialInput, attitudeControllerPID, positionControllerPID)

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
min_distance1 = 0
max_distance1 = 0

y_distance2 = []
max_distance2 = 0
y_distance3 = []
max_distance3 = 0

x_pos1 = []
x_pos1_max = 0
x_pos1_min = 0
x_pos2 = []
x_pos2_max = 0
x_pos2_min = 10

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
arena.set_zlim(-1, 12)

drone1Position = arena.scatter([position_drone1[0]], [position_drone1[1]], [position_drone1[2]], color="red")
drone2Position = arena.scatter([position_drone2[0]], [position_drone2[1]], [position_drone2[2]], color="green")
# drone3Position = arena.scatter([position_drone3[0]], [position_drone3[1]], [position_drone3[2]], color="blue")

view.show()

for i in range(5000):
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
    print('Drone1 targetPos', Drone1.position)
    targetPos1 = [Drone1.position[0], Drone1.position[1], Drone1.position[2]]
    AR1.controlPosition(targetPos1)
    AR1.updateState()
    newPos1 = (AR1.position[0], AR1.position[1], AR1.position[2])
    print('newPos1', newPos1)
    Drone1.position = (newPos1)

    Drone2.calculateVelocity(Drone2.SwarmPotentialForce)
    print('Drone2 Velocity', Drone2.velocity)
    Drone2.move()
    print('Drone2 targetPos', Drone2.position)
    targetPos2 = [Drone2.position[0], Drone2.position[1], Drone2.position[2]]
    AR2.controlPosition(targetPos2)
    AR2.updateState()
    newPos2 = (AR2.position[0], AR2.position[1], AR2.position[2])
    print('newPos2', newPos2)
    Drone2.position = (newPos2)


    # Drone3.calculateVelocity(Drone3.SwarmPotentialForce)
    # print('Drone3 Velocity', Drone3.velocity)
    # Drone3.move()
    # print('Drone3 Position', Drone3.position)

    plt.pause(0.01)
    drone1Position._offsets3d = ([AR1.position[0]], [AR1.position[1]], [AR1.position[2]])
    drone2Position._offsets3d = ([AR2.position[0]], [AR2.position[1]], [AR2.position[2]])
    # drone3Position._offsets3d = ([Drone3.position[0]], [Drone3.position[1]], [Drone3.position[2]])
    plt.draw()

    x_time.append(i/100)
    ## Update Graph

    newDis1 = SPF.getDistance(Drone1.index, Drone2.index, Drones)[1]
    y_distance1.append(newDis1)
    dis1.set_ydata(y_distance1)
    dis1.set_xdata(x_time)
    ax1.set_xlim(0, i/100)
    if newDis1 > max_distance1:
        max_distance1 = newDis1
    if newDis1 < min_distance1:
        min_distance1 = newDis1
    ax1.set_ylim(min_distance1 - 1, max_distance1 + 1)

    # newDis2 = SPF.getDistance(Drone2.index, Drone3.index, Drones)[1]
    x_pos1.append(Drone1.position[0])
    dis2.set_ydata(x_pos1)
    dis2.set_xdata(x_time)
    ax2.set_xlim(0, i/100)
    if Drone1.position[0] > x_pos1_max:
        x_pos1_max = Drone1.position[0]
    if Drone1.position[0] < x_pos1_min:
        x_pos1_min = Drone1.position[0]
    ax2.set_ylim(x_pos1_min-1, x_pos1_max+1)

    x_pos2.append(Drone2.position[0])
    dis3.set_ydata(x_pos2)
    dis3.set_xdata(x_time)
    ax3.set_xlim(0, i/100)
    if Drone2.position[0] > x_pos2_max:
        x_pos2_max = Drone2.position[0]
    if Drone2.position[0] < x_pos2_min:
        x_pos2_min = Drone2.position[0]
    ax3.set_ylim(x_pos2_min-1, x_pos2_max+1)

    report.canvas.draw()
    report.canvas.flush_events()

plt.pause(20)
