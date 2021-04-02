import matplotlib.pyplot as plt
import math
import sys
sys.path.append('../')
from Quadrotor import Quadrotor
from Bird import Bird
from Ship import Ship
from Report import Report
from FlyHistoryReport import FlyHistory
sys.path.append('../')
from Agent import Agent
from Obstacle import Obstacle
from Target import Target
from SwarmController import SwarmController

# Build Object for Attitude and Position Controller
specs = {"mass": 0.445, "inertia": [
    0.0027, 0.0029, 0.0053], "armLength": 0.125}
initialState = [[0.0, 0.0, 5.0], [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
initialState2 = [[-1.0, 1.0, 5.0], [0.0, 0.0, 0.0],
                 [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
initialState3 = [[7.0, -5.0, 5.0], [0.0, 0.0, 0.0],
                 [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]

initialInput = [0.0, 0.0, 0.0, 0.0]
attitudeControllerPID = [[1.43, 0, 0.13],  # PID phi
                         [1.52, 0, 0.14],  # PID theta
                         [2.43, 0, 0.26],  # PID psi
                         [0, 0, 0]]  # PID z dot
positionControllerPID = [[0, 0, 0],  # PID x
                         [0, 0, 0],  # PID y
                         [0, 0, 0]]  # PID z
AR1 = Quadrotor(0, "AR1", specs, initialState,
                   initialInput, attitudeControllerPID, positionControllerPID)
AR2 = Quadrotor(1, "AR2", specs, initialState2,
                   initialInput, attitudeControllerPID, positionControllerPID)
AR3 = Quadrotor(2, "AR3", specs, initialState3,
                   initialInput, attitudeControllerPID, positionControllerPID)
Target1 = Ship([4, 4, 0], 0.75)
Obstacle1 = Bird([1, 1.1, 5])

# For Plotting System Response
Report1 = Report(AR1)
Report2 = Report(AR2)
# Report3 = Report(AR3)

FlyReport = FlyHistory([-6,6],[-6,6],[3,5])

# Build Object for Swarm Controller
TPFconfig = {"damping_factor": 1, "gain":1, "target_detecting_range":1}
OPFconfig = {"positiveGain1": 1, "positiveGain2":1, "detecting_range": 1}
SPFconfig = {"min_allowable_dist": 10}
SwarmController1 = SwarmController(TPFconfig, OPFconfig, SPFconfig)
AR1.connectToSwarmController(SwarmController1)
AR2.connectToSwarmController(SwarmController1)
# AR3.connectToSwarmController(SwarmController1)
Target1.connectToSwarmController(SwarmController1)
Obstacle1.connectToSwarmController(SwarmController1)

for i in range(1000):
    SwarmController1.calculateAgentsForces()
    AR1.controlSwarm(SwarmController1)
    AR2.controlSwarm(SwarmController1)

    # AR limit ?
    AR1.updateState()
    AR2.updateState()

    # Plot Data
    Report1.updateReport(AR1.getState(), AR1.thrust, AR1.moments)
    Report2.updateReport(AR2.getState(), AR2.thrust, AR2.moments)

FlyReport.addObject(Report1.x, Report1.y, Report1.z, 0.25, 'g', 1)
# FlyReport.addObject(Report2.x, Report2.y, Report2.z, 0.25, 'red', 1)
# FlyReport.addObject(Report3.x, Report3.y, Report3.z, 0.25, 'blue', 1)

Report1.generateReport()
Report2.generateReport()
# FlyReport.draw()

plt.pause(20)
 