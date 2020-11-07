from Agent import Agent
from Target import Target
from TargetPotentialField import TargetPotentialField

Drones = []
Drone1 = Agent((0, 0, 0), 1)
Drones.append(Drone1)

Targets = []
Ship = Target((9, 9, 9))
Targets.append(Ship)

TPF = TargetPotentialField(1, 1, 1)

for i in range(0, 20):
    print('------- ITERATION ', i, '-----------')
    Drone1.TargetPotentialForce = TPF.calculate_target_force(0, 0, Drones, Targets)

    print('Drone1 TPF = ', Drone1.TargetPotentialForce, '\n')
    print(Drone1.calculateVelocity(Drone1.TargetPotentialForce))
    print('Drone1 Velocity', Drone1.velocity)
    Drone1.move()
    print('Drone1 Position', Drone1.position)

    print('\n\n')

