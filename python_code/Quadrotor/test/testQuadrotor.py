import matplotlib.pyplot as plt
import math
import sys
sys.path.append('../')
from Simulator import Simulator
from Report import Report
from Quadrotor import Quadrotor

specs = {"mass": 1.25, "inertia": [0.0232, 0.0232, 0.0232], "armLength": 0.265}
initialState = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
initialState2 = [[5.0, 5.0, 0.0], [0.0, 0.0, 0.0],
                 [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
initialState3 = [[7.0, -5.0, 0.0], [0.0, 0.0, 0.0],
                 [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]

initialInput = [0.0, 0.0, 0.0, 0.0]
attitudeControllerPID = [[6.84169791, 0, 1.13965542],  # PID phi
                         [1.51433707, 0, 1.06242888],  # PID theta
                         [12.00891182, 0.0, 1.14173959],  # PID psi
                         [84.16079374, 0.08603913, 0.77345114]]  # PID z dot

positionControllerPID = [[51.70660113, 50.22129217, 164.14666466],  # PID x
                         [20.79524657, 10, 0.10065972],  # PID y
                         [20.87769422, 1.27716434, 10.23173615]]  # PID z

Tello1 = Quadrotor("Tello1", specs, initialState,
                   initialInput, attitudeControllerPID, positionControllerPID)
Tello2 = Quadrotor("Tello2", specs, initialState2,
                   initialInput, attitudeControllerPID, positionControllerPID)
Tello3 = Quadrotor("Tello3", specs, initialState3,
                   initialInput, attitudeControllerPID, positionControllerPID)

Report1 = Report(Tello1)
Report2 = Report(Tello2)
Report3 = Report(Tello3)

# Show Quadrotor moving
plt.ion()
quadrotorView = plt.figure()
quadrotorArena = quadrotorView.add_subplot(111, projection='3d')

quadrotorArena.set_xlabel('X (meter)')
quadrotorArena.set_ylabel('Y (meter)')
quadrotorArena.set_zlabel('Z (meter)')

quadrotorArena.set_xlim(-1, 11)
quadrotorArena.set_ylim(-1, 11)
quadrotorArena.set_zlim(-2, 15)

# Build quadrotor in simulator
drone1Simulator = Simulator(Tello1, quadrotorArena)
# drone2Simulator = Simulator(Tello2, quadrotorArena)
# drone3Simulator = Simulator(Tello3, quadrotorArena)

drone1Simulator.initialDrawing("red", "blue")
# drone2Simulator.initialDrawing("yellow", "green")
# drone3Simulator.initialDrawing("green", "red")

# quadrotorView.show()

for i in range(200):
    print('-------Time:', Tello1.t, '-------')
    print('---------------------------------')

    # [phi, theta, psi, zdot] (degree)
    # Tello1.controlAttitude([-20, 0, 0, 0])
    # # Tello2.controlAttitude([10, 0, 0, 5])
    # # Tello3.controlAttitude([0, 0, 30, 3])
    Tello1.controlPosition([0, 1, 0])

    # Tello2.controlPosition([6,6,0])
    # Tello3.controlPosition([0,0,0])

    # Tello limit ?
    Tello1.updateState()
    print('y velocity= ', Tello1.position_dot[1])
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
