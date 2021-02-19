import matplotlib.pyplot as plt
import math
import sys
sys.path.append('../')
from Simulator import Simulator
from Report import Report
from Quadrotor import Quadrotor
from Ship import Ship
from Bird import Bird
sys.path.append('../')
from Agent import Agent
from Target import Target
from Obstacle import Obstacle   
from SwarmController import SwarmController

# Build Object for Attitude and Position Controller
specs = {"mass": 1.25, "inertia": [0.0232, 0.0232, 0.0232], "armLength": 0.265}
initialState = [[0.0, 0.0, 5.0], [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
initialState2 = [[-1.0, 1.0, 5.0], [0.0, 0.0, 0.0],
                 [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
initialState3 = [[7.0, -5.0, 0.0], [0.0, 0.0, 0.0],
                 [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]

initialInput = [0.0, 0.0, 0.0, 0.0]
attitudeControllerPID = [[0, 0, 0.88616771],  # PID phi
                                 [0, 0, 0.88616339],  # PID theta
                                 [0, 0, 0.88616098],  # PID psi
                                 [96.17007323, 7.35729734, 0]]  # PID z dot

positionControllerPID = [[27.83997997, 70.32934802, 4.70222228],  # PID x
                        [27.83997036, 68.08563516, 47.6545511],  # PID y
                        [0.84007563, 88.32316777, 68.43428605]]  # PID z

Tello1 = Quadrotor(0, "Tello1", specs, initialState,
                   initialInput, attitudeControllerPID, positionControllerPID)
Tello2 = Quadrotor(1, "Tello2", specs, initialState2,
                   initialInput, attitudeControllerPID, positionControllerPID)
Tello3 = Quadrotor(2, "Tello3", specs, initialState3,
                   initialInput, attitudeControllerPID, positionControllerPID)
Target1 = Ship([9,9,0], 0.75)
Obstacle1 = Bird([1,1.3,5])

# For Plotting System Response
Report1 = Report(Tello1)
# Report2 = Report(Tello2)
# Report3 = Report(Tello3)

# Build Object for Swarm Controller
TPFconfig = {"damping_factor": 1, "gain":1, "target_detecting_range":1}
OPFconfig = {"positiveGain1": 1, "positiveGain2":1, "detecting_range": 1}
SPFconfig = {"min_allowable_dist": 10}
SwarmController1 = SwarmController(TPFconfig, OPFconfig, SPFconfig)
Tello1.connectToSwarmController(SwarmController1)
# Tello2.connectToSwarmController(SwarmController1)
# Tello2.connectToSwarmController(SwarmController1)
# Tello3.connectToSwarmController(SwarmController1)
Target1.connectToSwarmController(SwarmController1)
Obstacle1.connectToSwarmController(SwarmController1)

# Show Quadrotor moving
plt.ion()
quadrotorView = plt.figure()
quadrotorArena = quadrotorView.add_subplot(111, projection='3d')

quadrotorArena.set_xlabel('X (meter)')
quadrotorArena.set_ylabel('Y (meter)')
quadrotorArena.set_zlabel('Z (meter)')

quadrotorArena.set_xlim(-1, 12)
quadrotorArena.set_ylim(-1, 12)
quadrotorArena.set_zlim(0, 10)

# Build object in simulator
drone1Simulator = Simulator(Tello1, quadrotorArena)
# drone2Simulator = Simulator(Tello2, quadrotorArena)
# drone3Simulator = Simulator(Tello3, quadrotorArena)
shipSimulator = Simulator(Target1, quadrotorArena)
birdSimulator = Simulator(Obstacle1, quadrotorArena)

drone1Simulator.initialDrawing("red", "blue")
# drone2Simulator.initialDrawing("yellow", "green")
# drone3Simulator.initialDrawing("green", "red")
shipSimulator.initialDrawing("black", "black")
birdSimulator.initialDrawing("brown", "brown")

quadrotorView.show()

for i in range(500):
    print('-------Time:', Tello1.t, '-------')
    print('---------------------------------')

    # [phi, theta, psi, zdot] (degree)
    # Tello1.controlAttitude([0, 0, 45, 0])
    # Tello2.controlAttitude([10, 0, 0, 5])
    # Tello3.controlAttitude([0, 0, 30, 3])

    # Tello1.controlPosition([0.7, 1, 5])
    # Tello2.controlPosition([6,6,0])
    # Tello3.controlPosition([0,0,0])

    SwarmController1.calculateAgentsForces()
    Tello1.controlSwarm(SwarmController1)
    # Tello2.controlSwarm(SwarmController1)

    # Tello limit ?
    Tello1.updateState()
    # Tello2.updateState()
    # print('degree =', Tello1.angles*180/math.pi)
    # print('velocity= ', Tello1.position_dot)
    # Tello2.updateState()
    # Tello3.updateState()

    # Plot Data
    Report1.updateReport(Tello1.getState(), Tello1.thrust, Tello1.moments)
    # Report2.updateReport(Tello2.getState(), Tello2.thrust, Tello2.moments)
    # Report3.updateReport(Tello3.getState(), Tello3.thrust, Tello3.moments)

    # Quadrotor View
    drone1Simulator.updateDrawing()
    # drone2Simulator.updateDrawing()
    # drone3Simulator.updateDrawing()

# for i in range(200):
#     print('-------Time:', Tello1.t, '-------')
#     print('---------------------------------')

#     # [phi, theta, psi, zdot] (degree)
#     Tello1.controlAttitude([0, 0, 0, 0])
#     Tello1.updateState()
#     print('y velocity= ', Tello1.position_dot[1])
#     Report1.updateReport(Tello1.getState(), Tello1.thrust, Tello1.moments)
#     drone1Simulator.updateDrawing()
      
# for i in range(200):
#     print('-------Time:', Tello1.t, '-------')
#     print('---------------------------------')

#     # [phi, theta, psi, zdot] (degree)
#     Tello1.controlAttitude([20, 0, 0, 0])
#     Tello1.updateState()
#     print('y velocity= ', Tello1.position_dot[1])
#     Report1.updateReport(Tello1.getState(), Tello1.thrust, Tello1.moments)
#     drone1Simulator.updateDrawing()

Report1.generateReport()
# Report2.generateReport()
# Report3.generateReport()

plt.pause(20)
