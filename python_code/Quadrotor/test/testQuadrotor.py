import matplotlib.pyplot as plt
import math
import sys
sys.path.append('../')
from Simulator import Simulator
from Report import Report
from Quadrotor import Quadrotor
from QuadrotorPEM import QuadrotorPEM
from Ship import Ship
from Bird import Bird
sys.path.append('../')
from Agent import Agent
from Target import Target
from Obstacle import Obstacle   
from SwarmController import SwarmController

# Build Object for Attitude and Position Controller
specs = {"mass": 0.445, "inertia": [0.0027, 0.0029, 0.0053], "armLength": 0.125}
initialState = [[0.0, 0.0, 1.0], [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
initialState2 = [[1.0, 4.0, 5.0], [0.0, 0.0, 0.0],
                 [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
initialState3 = [[7.0, -5.0, 0.0], [0.0, 0.0, 0.0],
                 [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]

initialInput = [0.0, 0.0, 0.0, 0.0] # input Thrust, input Roll, input Pitch, input Yaw
positionControllerPID = [[27.83997997, 70.32934802, 4.70222228],  # PID x
                        [27.83997036, 68.08563516, 47.6545511],  # PID y
                        [0.09, 1.59, 7.06]]  # PID z

AR1 = QuadrotorPEM(0, "AR1", specs, initialState,
                   initialInput, positionControllerPID)
Target1 = Ship([4,4,0], 0.75)
Obstacle1 = Bird([1,1.1,5])

# For Plotting System Response
Report1 = Report(AR1)
# Report2 = Report(AR2)
# Report3 = Report(AR3)

# Build Object for Swarm Controller
TPFconfig = {"damping_factor": 1, "gain":1, "target_detecting_range":1}
OPFconfig = {"positiveGain1": 1, "positiveGain2":1, "detecting_range": 1}
SPFconfig = {"min_allowable_dist": 10}
SwarmController1 = SwarmController(TPFconfig, OPFconfig, SPFconfig)
AR1.connectToSwarmController(SwarmController1)
# AR2.connectToSwarmController(SwarmController1)
# AR3.connectToSwarmController(SwarmController1)
Target1.connectToSwarmController(SwarmController1)
Obstacle1.connectToSwarmController(SwarmController1)

# Show Quadrotor moving
plt.ion()
quadrotorView = plt.figure()
quadrotorArena = quadrotorView.add_subplot(111, projection='3d')

quadrotorArena.set_xlabel('X (meter)')
quadrotorArena.set_ylabel('Y (meter)')
quadrotorArena.set_zlabel('Z (meter)')

quadrotorArena.set_xlim(-1, 6)
quadrotorArena.set_ylim(-1, 6)
quadrotorArena.set_zlim(0, 10)

# Build object in simulator
drone1Simulator = Simulator(AR1, quadrotorArena)
# drone2Simulator = Simulator(AR2, quadrotorArena)
# drone3Simulator = Simulator(AR3, quadrotorArena)
shipSimulator = Simulator(Target1, quadrotorArena)
birdSimulator = Simulator(Obstacle1, quadrotorArena)

drone1Simulator.initialDrawing("red", "blue")
# drone2Simulator.initialDrawing("yellow", "green")
# drone3Simulator.initialDrawing("green", "red")
shipSimulator.initialDrawing("black", "black")
birdSimulator.initialDrawing("brown", "brown")

quadrotorView.show()

for i in range(5):
    print('-------Time:', AR1.t, '-------')
    print('---------------------------------')

    AR1.controlPosition([10, 0, 1])
    # AR2.controlPosition([6,6,0])
    # AR3.controlPosition([0,0,0])

    # SwarmController1.calculateAgentsForces()
    # AR1.controlSwarm(SwarmController1)
    # AR2.controlSwarm(SwarmController1)


    # Plot Data
    Report1.updateReport(AR1.getState(), AR1.getInput())
    # Report2.updateReport(AR2.getState(), AR2.thrust, AR2.moments)
    # Report3.updateReport(AR3.getState(), AR3.thrust, AR3.moments)

    # Quadrotor View
    drone1Simulator.updateDrawing()
    # drone2Simulator.updateDrawing()
    # drone3Simulator.updateDrawing()

Report1.generateReport()
# Report2.generateReport()
# Report3.generateReport()

plt.pause(20)
