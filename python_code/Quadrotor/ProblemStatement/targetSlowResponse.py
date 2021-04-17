import matplotlib.pyplot as plt
import math
import sys
sys.path.append('../')
from Report import Report
from QuadrotorARSim import QuadrotorARSim
from Ship import Ship
sys.path.append('../')
from Agent import Agent
from Target import Target
from SwarmController import SwarmController

# Build Object for Attitude and Position Controller
specs = {"mass": 0.445, "inertia": [
    0.0027, 0.0029, 0.0053], "armLength": 0.125}
initialState = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
initialState2 = [[-1.0, 1.0, 0.0], [0.0, 0.0, 0.0],
                 [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
initialState3 = [[7.0, -5.0, 0.0], [0.0, 0.0, 0.0],
                 [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]

initialInput = [0.0, 0.0, 0.0, 0.0]
attitudeControllerPID = [[1.43, 0, 0.13],  # PID phi
                         [1.52, 0, 0.14],  # PID theta
                         [2.43, 0, 0.26],  # PID psi
                         [88.02, 44.5, 0 ]]  # PID z dot
positionControllerPID = [[0, 0, 0],  # PID x
                         [0, 0, 0],  # PID y
                         [5, 1, 2]]  # PID z
AR1 = QuadrotorARSim(0, "AR1", specs, initialState,
                   initialInput, attitudeControllerPID, positionControllerPID)
AR2 = QuadrotorARSim(1, "AR2", specs, initialState2,
                   initialInput, attitudeControllerPID, positionControllerPID)
AR3 = QuadrotorARSim(2, "AR3", specs, initialState3,
                   initialInput, attitudeControllerPID, positionControllerPID)
Target1 = Ship([4, 4, 0], 0.75)

# For Plotting System Response
Report1 = Report(AR1)
Report2 = Report(AR2)
Report3 = Report(AR3)

# Build Object for Swarm Controller
TPFconfig = {"damping_factor": 1, "gain":1, "target_detecting_range":1}
OPFconfig = {"positiveGain1": 1, "positiveGain2":1, "detecting_range": 1}
SPFconfig = {"min_allowable_dist": 10}
SwarmController1 = SwarmController(TPFconfig, OPFconfig, SPFconfig)
AR1.connectToSwarmController(SwarmController1)
AR2.connectToSwarmController(SwarmController1)
AR3.connectToSwarmController(SwarmController1)
Target1.connectToSwarmController(SwarmController1)

for iteration in range (100):
    print('-------Time:', AR1.t, '-------')

    # SwarmController1.calculateAgentsForces()
    # AR1.controlSwarm(SwarmController1)
    # AR2.controlSwarm(SwarmController1)
    # AR3.controlSwarm(SwarmController1)
    AR1.controlPosition([0,0,1])
    AR2.controlPosition([0,0,1])
    AR3.controlPosition([0,0,1])

    AR1.updateState()
    AR2.updateState()
    AR3.updateState()

    Report1.updateReport(AR1.getState(), AR1.thrust, AR1.moments)
    Report2.updateReport(AR2.getState(), AR2.thrust, AR2.moments)
    Report3.updateReport(AR3.getState(), AR3.thrust, AR3.moments)


Report1.generateReport()
Report2.generateReport()
Report3.generateReport()

plt.pause(20)