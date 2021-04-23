import sys 
sys.path.append('../')
import math
from VirtualStructure import VirtualStructure

radianToDegree = 180/math.pi
degreeToRadian = math.pi/180

VS = VirtualStructure("triangle", (0, 0, 0*degreeToRadian)) # heading in radian
VS.setFRPPosition([0,0,1])

Quadrotor1Pos = (1, 0, 1)
Quadrotor2Pos = (0.5, -0.5, 1)
Quadrotor3Pos = (-0.5, -0.5, 1)
currentRealPointPos = [Quadrotor1Pos, Quadrotor2Pos, Quadrotor3Pos]
VS.setRealPointPositions(currentRealPointPos)

APFForce = (1, 1, 0)
newTargetPos = VS.calculateNewVSPoint(APFForce)

print("new target pos", newTargetPos)

print(VS.VSPPositions)

