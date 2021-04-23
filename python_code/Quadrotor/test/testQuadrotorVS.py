import matplotlib.pyplot as plt
import math
import numpy as np
import sys
sys.path.append('../')
from VirtualStructure import VirtualStructure
from Quadrotor import Quadrotor
from Bird import Bird
from Ship import Ship
from Report import Report
from FlyHistoryReport import FlyHistoryReport
from DynamicGraph import DynamicGraph
sys.path.append('../')
from Agent import Agent
from Obstacle import Obstacle
from Target import Target
from SwarmController import SwarmController

radianToDegree = 180/math.pi
degreeToRadian = math.pi/180

# Build Object for Swarm Controller
TPFconfig = {"damping_factor": 1, "gain":1, "target_detecting_range":1}
OPFconfig = {"positiveGain1": 1, "positiveGain2":1, "detecting_range": 2}
SPFconfig = {"min_allowable_dist": 10}
SwarmController1 = SwarmController(TPFconfig, OPFconfig, SPFconfig)

# Virtual Structure
VS = VirtualStructure("triangle", (0,0,0*degreeToRadian))
VS.setFRPPosition([0,0,5])
VS.connectToSwarmController(SwarmController1)

# Build Quadrotor
specs = {"mass": 0.445, "inertia": [
    0.0027, 0.0029, 0.0053], "armLength": 0.125}
initialState = [[0.0, 0.0, 5.0], [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
initialState2 = [[-0.5, -0.5, 5.0], [0.0, 0.0, 0.0],
                 [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
initialState3 = [[-1.5, -0.5, 5.0], [0.0, 0.0, 0.0],
                 [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]

initialInput = [0.0, 0.0, 0.0, 0.0]
attitudeControllerPID = [[1.43, 0, 0.13],  # PID phi
                        [1.52, 0, 0.14],  # PID theta
                        [2.43, 0, 0.26],  # PID psi
                        [48.49, 14.29, 0.0]]  # PID z dot
positionControllerPID = [[1.59, 56.7, 4.77],  # PID x
                        [0.96, 88.38, 73.36],    # PID y
                        [0.08, 40.31, 92.08]]  # PID z

AR1 = Quadrotor(0, "AR1", specs, initialState,
                   initialInput, attitudeControllerPID, positionControllerPID)
AR2 = Quadrotor(1, "AR2", specs, initialState2,
                   initialInput, attitudeControllerPID, positionControllerPID)
AR3 = Quadrotor(2, "AR3", specs, initialState3,
                   initialInput, attitudeControllerPID, positionControllerPID)

Target1 = Ship([10, 10, 0], 5)
Target1.connectToSwarmController(SwarmController1)

Obstacle1 = Bird([5, 6, 5])
Obstacle1.connectToSwarmController(SwarmController1)

Obstacle2 = Bird([5, 5, 5])
Obstacle2.connectToSwarmController(SwarmController1)

Obstacle3 = Bird([5, 4, 5])
Obstacle3.connectToSwarmController(SwarmController1)

Obstacle4 = Bird([5, 9, 5])
Obstacle4.connectToSwarmController(SwarmController1)

Obstacle5 = Bird([4, 2, 5])
Obstacle5.connectToSwarmController(SwarmController1)

Obstacle6 = Bird([7, 9, 5])
Obstacle6.connectToSwarmController(SwarmController1)

Obstacle7 = Bird([8, 9, 5])
Obstacle7.connectToSwarmController(SwarmController1)

Obstacle8 = Bird([0, 2, 5])
Obstacle8.connectToSwarmController(SwarmController1)

# For Plotting System Response
Report1 = Report(AR1)
Report2 = Report(AR2)
Report3 = Report(AR3)

# PSI_ERR_GRAPH = DynamicGraph(0.01)
# APF_GRAPH = DynamicGraph(0.01)
# APF_GRAPH.createPlot(0, 'X', 0, 'g-')
# APF_GRAPH.createPlot(1, 'Y', 0, 'r-')
# APF_GRAPH.createPlot(2, 'Z', 0, 'b-')
# PSI_ERR_GRAPH.createPlot(0, 'PSI ERR AR1', 1, 'g-')
# PSI_ERR_GRAPH.createPlot(1, 'PSI ERR AR2', 2, 'r-')
# PSI_ERR_GRAPH.createPlot(2, 'PSI ERR AR3', 3, 'b-')

# Fly History
# plt.ion()
quadrotorView = plt.figure()
quadrotorArena = quadrotorView.add_subplot(111, projection='3d')

quadrotorArena.set_xlabel('X (meter)')
quadrotorArena.set_ylabel('Y (meter)')
quadrotorArena.set_zlabel('Z (meter)')

quadrotorArena.set_xlim(-2, 14)
quadrotorArena.set_ylim(-2, 14)
quadrotorArena.set_zlim(4, 6)

FlyHistoryReport1 = FlyHistoryReport(AR1, quadrotorArena)

# Draw Obstacle and Target
## Target (GREEN)
FlyHistoryReport1.addObject(10, 10, 5, 0.05, 'g', 1)

## Obstacle (RED)
FlyHistoryReport1.addObject(5, 6, 5, 0.05, 'r', 1)
FlyHistoryReport1.addObject(5, 5, 5, 0.05, 'r', 1)
FlyHistoryReport1.addObject(5, 4, 5, 0.05, 'r', 1)
FlyHistoryReport1.addObject(5, 9, 5, 0.05, 'r', 1)
FlyHistoryReport1.addObject(4, 2, 5, 0.05, 'r', 1)
FlyHistoryReport1.addObject(7, 9, 5, 0.05, 'r', 1)
FlyHistoryReport1.addObject(8, 9, 5, 0.05, 'r', 1)
FlyHistoryReport1.addObject(0, 2, 5, 0.05, 'r', 1)

# History Data
AR1PositionHistory = np.array([[AR1.position[0], AR1.position[1], AR1.position[2]]])
AR2PositionHistory = np.array([[AR2.position[0], AR2.position[1], AR2.position[2]]])
AR3PositionHistory = np.array([[AR3.position[0], AR3.position[1], AR3.position[2]]])

VSPositionHistory = np.array([VS.FRPPosition])

# quadrotorView.show()

for i in range(8000):
    # Virtual Structure calculate, freq 1/10 
    if i%5 == 0:
        currentRealPointPos = [AR1.position, AR2.position, AR3.position]
        VS.setRealPointPositions(currentRealPointPos)
        SwarmController1.calculateAgentsForces()
        APFForce = SwarmController1.agents[0].calculate_total_force()
        newTargetPos = VS.calculateNewVSPoint(APFForce)
        yawTarget1 = np.arctan2(newTargetPos[0][1], newTargetPos[0][0])
        yawTarget2 = np.arctan2(newTargetPos[1][1], newTargetPos[1][0])
        yawTarget3 = np.arctan2(newTargetPos[2][1], newTargetPos[2][0])

        print('------------------------------')
        print('---- VS - REAL WORLD POSITION')
        print('real world pos 1', currentRealPointPos[0])
        print('real world pos 2', currentRealPointPos[1])
        print('real world pos 3', currentRealPointPos[2])
        print("APF Force", APFForce)
        print("VS RP", VS.FRPPosition)
        print('NEW TARGET POS 1', newTargetPos[0])
        print('NEW TARGET POS 2', newTargetPos[1])
        print('NEW TARGET POS 3', newTargetPos[2])
        print('')
        print('------------------------------')

    AR1.controlPositionYaw((newTargetPos[0][0], newTargetPos[0][1], newTargetPos[0][2]), yawTarget1)
    AR2.controlPositionYaw((newTargetPos[1][0], newTargetPos[1][1], newTargetPos[1][2]), yawTarget2)
    AR3.controlPositionYaw((newTargetPos[2][0], newTargetPos[2][1], newTargetPos[2][2]), yawTarget3)

    # PSI_ERR_GRAPH.updatePlotData(0, AR1.psi_err*radianToDegree)
    # PSI_ERR_GRAPH.updatePlotData(1, AR2.psi_err*radianToDegree)
    # PSI_ERR_GRAPH.updatePlotData(2, AR3.psi_err*radianToDegree)
    # PSI_ERR_GRAPH.updatePlot()
    # APF_GRAPH.updatePlotData(0, APFForce[0])
    # APF_GRAPH.updatePlotData(1, APFForce[1])
    # APF_GRAPH.updatePlotData(2, APFForce[2])
    # APF_GRAPH.updatePlot()

    AR1.updateState()
    AR2.updateState()
    AR3.updateState()

    # Plot Data
    Report1.updateReport(AR1.getState(), AR1.thrust, AR1.moments)
    Report2.updateReport(AR2.getState(), AR2.thrust, AR2.moments)
    Report3.updateReport(AR3.getState(), AR3.thrust, AR3.moments)

    if i%5 == 0:
        AR1PositionHistory = np.concatenate(
            (AR1PositionHistory, np.array([[
                AR1.position[0],
                AR1.position[1],
                AR1.position[2],
            ]]))
            )
        AR2PositionHistory = np.concatenate(
            (AR2PositionHistory, np.array([[
                AR2.position[0],
                AR2.position[1],
                AR2.position[2],
            ]]))
            )
        AR3PositionHistory = np.concatenate(
            (AR3PositionHistory, np.array([[
                AR3.position[0],
                AR3.position[1],
                AR3.position[2],
            ]]))
            )
        VSPositionHistory = np.concatenate(
            (VSPositionHistory, np.array([
                VS.FRPPosition
            ]))
        )

    # AR1History.addObject(AR1.position[0], AR1.position[1], AR1.position[2], 0.0125, 'b', 1)
    # AR2History.addObject(AR2.position[0], AR2.position[1], AR2.position[2], 0.0125, 'b', 1)
    # AR3History.addObject(AR3.position[0], AR3.position[1], AR3.position[2], 0.0125, 'b', 1)

FlyHistoryReport1.addHistory(AR1PositionHistory[:,0], AR1PositionHistory[:,1], AR1PositionHistory[:,2], 0.01, 'b', 1)
FlyHistoryReport1.addHistory(AR2PositionHistory[:,0], AR2PositionHistory[:,1], AR2PositionHistory[:,2], 0.01, 'c', 1)
FlyHistoryReport1.addHistory(AR3PositionHistory[:,0], AR3PositionHistory[:,1], AR3PositionHistory[:,2], 0.01, 'k', 1)
FlyHistoryReport1.addHistory(VSPositionHistory[:,0], VSPositionHistory[:,1], VSPositionHistory[:,2], 0.01, 'y', 1)

Report1.generateReport()
Report2.generateReport()
Report3.generateReport()
plt.pause(100)

