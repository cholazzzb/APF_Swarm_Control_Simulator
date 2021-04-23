import matplotlib.pyplot as plt
import sys
sys.path.append('../')

from Quadrotor import Quadrotor
from FlyHistoryReport import FlyHistoryReport


specs = {"mass": 0.445, "inertia": [
    0.0027, 0.0029, 0.0053], "armLength": 0.125}
initialState = [[0.0, 0.0, 5.0], [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]

initialInput = [0.0, 0.0, 0.0, 0.0]
attitudeControllerPID = [[1.43, 0, 0.13],  # PID phi
                            [1.52, 0, 0.14],  # PID theta
                            [2.43, 0, 0.26],  # PID psi
                            [48.49, 14.29, 0.0]]  # PID z dot
positionControllerPID = [[323.35, 8.38, 177.91],  # PID x
                        [138.38, 126.43, 98.03],    # PID y
                        [0.08, 40.31, 92.08]]  # PID z

AR1 = Quadrotor(0, "AR1", specs, initialState,
                   initialInput, attitudeControllerPID, positionControllerPID)


plt.ion()
quadrotorView = plt.figure()
quadrotorArena = quadrotorView.add_subplot(111, projection='3d')

quadrotorArena.set_xlabel('X (meter)')
quadrotorArena.set_ylabel('Y (meter)')
quadrotorArena.set_zlabel('Z (meter)')

quadrotorArena.set_xlim(-1, 6)
quadrotorArena.set_ylim(-1, 6)
quadrotorArena.set_zlim(0, 10)

AR1History = FlyHistoryReport(AR1, quadrotorArena)
AR2History = FlyHistoryReport(AR1, quadrotorArena)

quadrotorView.show()

for i in range(10):
    AR2History.addObject(2, 1+i/100, 3, 0.025, 'b', 1)
    AR1History.addObject(1+i/100, 2, 3, 0.025, 'g', 1)
plt.pause(20)
