from Agent import Agent
from SwarmPotentialField import SwarmPotentialField

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

min_allowable_dist = 10

Drones = []
position_drone1 = [(45, -89, 8)]
Drone1 = Agent(0, position_drone1[0], 1.5)
Drones.append(Drone1)

position_drone2 = [(35, 108, 9)]
Drone2 = Agent(1, position_drone2[0], 1)
Drones.append(Drone2)

position_drone3 = [(14, 13, 0)]
Drone3 = Agent(2, position_drone3[0], 1)
Drones.append(Drone3)

# position_drone4 = [(35, 108, 9)]
# Drone4 = Agent(1, position_drone4[0], 1)
# Drones.append(Drone4)

SPF = SwarmPotentialField(min_allowable_dist)

for i in range(0, 50):
    print('------- ITERATION ', i, '-----------')

    Drone1.SwarmPotentialForce = SPF.calculate_total_swarm_field_force(Drone1.index, Drones)
    print('Drone1 SPF = ', Drone1.SwarmPotentialForce, '\n')
    Drone2.SwarmPotentialForce = SPF.calculate_total_swarm_field_force(Drone2.index, Drones)
    print('Drone2 SPF = ', Drone2.SwarmPotentialForce, '\n')
    Drone3.SwarmPotentialForce = SPF.calculate_total_swarm_field_force(Drone3.index, Drones)
    print('Drone3 SPF = ', Drone3.SwarmPotentialForce, '\n')
        
    print(Drone1.calculateVelocity(Drone1.SwarmPotentialForce))
    print('Drone1 Velocity', Drone1.velocity)
    Drone1.move()
    print('Drone1 Position', Drone1.position)
    position_drone1.append(Drone1.position)

    print('\n')

    print(Drone2.calculateVelocity(Drone2.SwarmPotentialForce))
    print('Drone2 Velocity', Drone2.velocity)
    Drone2.move()
    print('Drone2 Position', Drone2.position)
    position_drone2.append(Drone2.position)

    print('\n')

    print(Drone3.calculateVelocity(Drone3.SwarmPotentialForce))
    print('Drone3 Velocity', Drone3.velocity)
    Drone3.move()
    print('Drone3 Position', Drone3.position)
    position_drone3.append(Drone3.position)

    print('\n')

# Report Codes
def convertPosition(positions):
    x = []
    y = []
    z = []
    for position in positions:
        x.append(position[0])
        y.append(position[1])
        z.append(position[2])
    return [x, y, z]


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for color, mark, positions in [('r', 'o', position_drone1), ('b', '^', position_drone2), ('g', 'o', position_drone3)]:
    [xs, ys, zs] = convertPosition(positions)
    ax.scatter(xs, ys, zs, c=color, marker=mark)

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

plt.show()