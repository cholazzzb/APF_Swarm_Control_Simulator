from Agent import Agent
from Obstacle import Obstacle
from ObstaclePotentialField import ObstaclePotentialField

Drones = []
Drone1 = Agent(0, (0, 0, 0), 1)
Drones.append(Drone1)

Drone2 = Agent(1, (0, 0, 9), 1)
Drones.append(Drone2)

Obstacles = []
Coral = Obstacle((9, 9, 9))
Obstacles.append(Coral)

OPF = ObstaclePotentialField(1, 1, 20)

for i in range(0, 20):
    print('------- ITERATION ', i, '-----------')
    obstacle1_forces = OPF.calculate_obstacle_forces(Drones, Coral)
    Drone1.ObstaclePotentialForce = obstacle1_forces[Drone1.index]
    Drone2.ObstaclePotentialForce = obstacle1_forces[Drone2.index]

    print('Drone1 OPF = ', Drone1.ObstaclePotentialForce, '\n')
    print(Drone1.calculateVelocity(Drone1.ObstaclePotentialForce))
    print('Drone1 Velocity', Drone1.velocity)
    Drone1.move()
    print('Drone1 Position', Drone1.position)

    print('\n')

    print('Drone2 OPF = ', Drone2.ObstaclePotentialForce, '\n')
    print(Drone2.calculateVelocity(Drone2.ObstaclePotentialForce))
    print('Drone2 Velocity', Drone2.velocity)
    Drone2.move()
    print('Drone2 Position', Drone2.position)

    print('\n\n')

