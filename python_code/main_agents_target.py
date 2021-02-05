from Agent import Agent
from SwarmPotentialField import SwarmPotentialField
from Target import Target
from TargetPotentialField import TargetPotentialField
from Simulator import Simulator

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

Drones = []
position_drone1 = [(45, -89, 8)]
Drone1 = Agent(0, position_drone1[0], 1.5)
Drones.append(Drone1)

position_drone2 = [(35, 108, 9)]
Drone2 = Agent(1, position_drone2[0], 1)
Drones.append(Drone2)

Ships = []
position_ship1 = [(0,  0, 0)]
Ship1 = Target(position_ship1[0])
Ship1.setVelocity((2, 2, 0))
Ships.append(Ship1)

SPF = SwarmPotentialField(10)
TPF = TargetPotentialField(1,10,1)

for i in range(0,20):
    print('------- ITERATION ', i, '-----------')
    Drone1.SwarmPotentialForce = SPF.calculate_total_swarm_field_force(Drone1.index, Drones)
    Drone2.SwarmPotentialForce = SPF.calculate_total_swarm_field_force(Drone2.index, Drones)
    Drone1.TargetPotentialForce = TPF.calculate_target_force(Drone1.index, 0, Drones, Ships)
    Drone2.TargetPotentialForce = TPF.calculate_target_force(Drone2.index, 0, Drones, Ships)

    print('Drone1 pos', Drone1.position)
    print('FORCE')
    print('swarm force', Drone1.SwarmPotentialForce)
    print('target force', Drone1.TargetPotentialForce)
    Drone1.calculateVelocity(Drone1.calculate_total_force())
    print('Drone1 vel', Drone1.velocity)
    Drone1.move()
    print('Drone1 new pos', Drone1.position)

    print('Drone2 pos', Drone2.position)
    print('FORCE')
    # print('swarm force', Drone2.SwarmPotentialForce)
    print('target force', Drone2.TargetPotentialForce)
    Drone2.calculateVelocity(Drone2.calculate_total_force())
    print('Drone2 vel', Drone2.velocity)
    Drone2.move()
    print('Drone2 new pos', Drone2.position)


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

Result = Simulator()
Result.set_agent_entities(Drones)
Result.set_target_entities(Ships)

for color, mark, positions in Result.entities:
    [xs, ys, zs] = convertPosition(positions)
    ax.scatter(xs, ys, zs, c=color, marker=mark)

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

plt.show()