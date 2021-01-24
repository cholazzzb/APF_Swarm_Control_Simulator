import sys
sys.path.append('../')
from Quadrotor import Quadrotor
from Report import Report
from Simulator import Simulator
import matplotlib.pyplot as plt
import math

radianToDegree = 180/math.pi
degreeToRadian = math.pi/180

specs = {"mass": 1.25, "inertia": [0.0232, 0.0232, 0.0232], "armLength": 0.265}
initialState = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
initialState2 = [[5.0, 5.0, 0.0], [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
initialState3 = [[7.0, -5.0, 0.0], [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]

initialInput = [0.0, 0.0, 0.0, 0.0]
attitudeControllerPID = [[0.2, 0.0, 0.15],  # PID phi
                         [0.2, 0.0, 0.15],  # PID theta
                         [0.8, 0.0, 0.3],  # PID psi
                         [10.0, 0.2, 0.0]]  # PID z dot

Tello1 = Quadrotor("Tello1", specs, initialState, initialInput, attitudeControllerPID)
Tello2 = Quadrotor("Tello2", specs, initialState2, initialInput, attitudeControllerPID)
Tello3 = Quadrotor("Tello3", specs, initialState3, initialInput, attitudeControllerPID)

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

quadrotorArena.set_xlim(-1, 8)
quadrotorArena.set_ylim(-6, 6)
quadrotorArena.set_zlim(-2, 3)

# Build quadrotor in simulator
drone1Simulator = Simulator(Tello1, quadrotorArena)
drone2Simulator = Simulator(Tello2, quadrotorArena)
drone3Simulator = Simulator(Tello3, quadrotorArena)

drone1Simulator.initialDrawing("red", "blue")
drone2Simulator.initialDrawing("yellow", "green")
drone3Simulator.initialDrawing("green", "red")

quadrotorView.show()

for i in range(200):
    print('-------Time:', Tello1.t, '-------')
    print('---------------------------------')

    # [phi, theta, psi, zdot] (degree)
    Tello1.controlAttitude(
        [5*degreeToRadian, 5*degreeToRadian, 0*degreeToRadian, 5])
    Tello2.controlAttitude(
        [10*degreeToRadian, 0*degreeToRadian, 0*degreeToRadian, 5])
    Tello3.controlAttitude(
        [1*degreeToRadian, 10*degreeToRadian, 0*degreeToRadian, 0])

    # Tello limit ?
    Tello1.updateState()
    Tello2.updateState()
    Tello3.updateState()

    # Plot Data
    Report1.updateReport(Tello1.getState(), Tello1.thrust, Tello1.moments)   
    Report2.updateReport(Tello2.getState(), Tello2.thrust, Tello2.moments)
    Report3.updateReport(Tello3.getState(), Tello3.thrust, Tello3.moments)

    # Quadrotor View
    drone1Simulator.updateDrawing()
    drone2Simulator.updateDrawing()
    drone3Simulator.updateDrawing()

Report1.generateReport()
Report2.generateReport()
Report3.generateReport()

plt.pause(100)