from Agent import Agent
from Obstacle import Obstacle
from ObstaclePotentialField import ObstaclePotentialField
from Target import Target
from TargetPotentialField import TargetPotentialField
from Simulator import Simulator

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

Drones = []
position_drone1 = [(45, -89, 100)]
Drone1 = Agent(0, position_drone1[0], 1.5)
Drones.append(Drone1)

position_drone2 = [(0, 100, 100)]
Drone2 = Agent(1, position_drone2[0], 1)
Drones.append(Drone2)

Obstacles = []
Coral = Obstacle((0, 60, 50))
Coral2 = Obstacle((0, 50, 50))
Obstacles.append(Coral)
Obstacles.append(Coral2)

Ships = []
position_ship1 = [(0,  0, 0)]
Ship1 = Target(position_ship1[0])
Ship1.setVelocity((2, 2, 0))
Ships.append(Ship1)

OPF = ObstaclePotentialField(1, 1, 10)
TPF = TargetPotentialField(1, 1, 1)

for i in range(0, 100):
    print('------- ITERATION ', i, '-----------')
    obstacle1_forces = OPF.calculate_obstacle_forces(Drones, Coral)
    obstacle2_forces = OPF.calculate_obstacle_forces(Drones, Coral2) 
    Drone1.ObstaclePotentialForce = obstacle1_forces[Drone1.index]
    Drone2.ObstaclePotentialForce = obstacle1_forces[Drone2.index] 

    Drone1.TargetPotentialForce = TPF.calculate_target_force(
        Drone1.index, 0, Drones, Ships)
    Drone2.TargetPotentialForce = TPF.calculate_target_force(
        Drone2.index, 0, Drones, Ships)

    print('Drone1 TPF = ', Drone1.TargetPotentialForce, '\n')
    print('Drone1 OPF = ', Drone1.ObstaclePotentialForce, '\n')
    print('Drone1 Total Force = ', Drone1.calculate_total_force(), '\n')
    print(Drone1.calculateVelocity(Drone1.calculate_total_force()))
    print('Drone1 Velocity', Drone1.velocity)
    Drone1.move()
    print('Drone1 Position', Drone1.position)

    print('\n')

    print('Drone2 TPF = ', Drone2.TargetPotentialForce, '\n')
    print('Drone2 OPF = ', Drone2.ObstaclePotentialForce, '\n')
    print('Drone2 Total Force = ', Drone2.calculate_total_force(), '\n')
    print(Drone2.calculateVelocity(Drone2.calculate_total_force()))
    print('Drone2 Velocity', Drone2.velocity)
    Drone2.move()
    print('Drone2 Position', Drone2.position)

    print('\n\n')

    Ship1.move()
    Ship1.positionHistory.append(Ship1.position)


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
Result.set_obstacle_entities(Obstacles)
Result.set_target_entities(Ships)

for color, mark, positions in Result.entities:
    [xs, ys, zs] = convertPosition(positions)
    ax.scatter(xs, ys, zs, c=color, marker=mark)

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

plt.show()
