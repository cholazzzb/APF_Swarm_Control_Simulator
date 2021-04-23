import matplotlib.pyplot as plt
import math
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

# Build Quadrotor
specs = {"mass": 0.445, "inertia": [
    0.0027, 0.0029, 0.0053], "armLength": 0.125}
initialState = [[0.0, 0.0, 5.0], [0.0, 0.0, 0.0],
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
Target1 = Ship([4, 4, 0], 5)
Obstacle1 = Bird([1, 1.1, 5])

# For Plotting System Response
Report1 = Report(AR1)

# FlyReport = FlyHistoryReport([-6,6],[-6,6],[-6,6])

# Virtual Structure
VS = VirtualStructure("triangle", (0,0,0*degreeToRadian))
VS.setFRPPosition([0,0,5])

# Build Object for Swarm Controller
TPFconfig = {"damping_factor": 1, "gain":1, "target_detecting_range":1}
OPFconfig = {"positiveGain1": 1, "positiveGain2":1, "detecting_range": 1}
SPFconfig = {"min_allowable_dist": 10}
SwarmController1 = SwarmController(TPFconfig, OPFconfig, SPFconfig)

VS.connectToSwarmController(SwarmController1)
Target1.connectToSwarmController(SwarmController1)
Obstacle1.connectToSwarmController(SwarmController1)

positionGraph = DynamicGraph(0.01)
positionGraph.createPlot(0, 'X Pos AR1', AR1.position[0], 'g-')
positionGraph.createPlot(1, 'Y Pos AR1', AR1.position[1], 'r-')
positionGraph.createPlot(2, 'Psi Pos AR1', AR1.angles[2]*radianToDegree, 'b-')

plt.ion()

for i in range(1000):
    AR1.controlPositionYaw((0,1,5), 90)
    # if AR1.angles[2]*radianToDegree > -87 or AR1.angles[2]*radianToDegree < -93 :
    #     AR1.controlAttitude((0,0,-90,0))
    # else:
    #     AR1.controlAttitude((0, 30, -90, 0))
 
    positionGraph.updatePlotData(0, AR1.position[0])
    positionGraph.updatePlotData(1, AR1.position[1])
    positionGraph.updatePlotData(2, AR1.angles[2]*radianToDegree)
    positionGraph.updatePlot()

    AR1.updateState()
    
    # Plot Data
    Report1.updateReport(AR1.getState(), AR1.thrust, AR1.moments)
   
Report1.generateReport()
plt.pause(100)

