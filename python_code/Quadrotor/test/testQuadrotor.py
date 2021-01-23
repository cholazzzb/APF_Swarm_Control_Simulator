import sys
sys.path.append('../')
from Quadrotor import Quadrotor
from Report import Report
import matplotlib.pyplot as plt
import math

radianToDegree = 180/math.pi
degreeToRadian = math.pi/180

specs = {"mass": 1.25, "inertia": [0.0232, 0.0232, 0.0232], "armLength": 0.265}
initialState = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
initialState2 = [[5.0, 5.0, 0.0], [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
initialInput = [0.0, 0.0, 0.0, 0.0]
attitudeControllerPID = [[0.2, 0.0, 0.15],  # PID phi
                         [0.2, 0.0, 0.15],  # PID theta
                         [0.8, 0.0, 0.3],  # PID psi
                         [10.0, 0.2, 0.0]]  # PID z dot

Tello1 = Quadrotor("Tello1", specs, initialState, initialInput, attitudeControllerPID)
Tello2 = Quadrotor("Tello2", specs, initialState2, initialInput, attitudeControllerPID)

Report1 = Report(Tello1)
Report2 = Report(Tello2)

tello1State = Tello1.getState()
tello2State = Tello2.getState()

tello1BodyPosition = Tello1.getBodyPosition()
# Position Arena
tello1X = [tello1State[0][0]]
tello1Y = [tello1State[0][1]]
tello1Z = [tello1State[0][2]]

# Show Quadrotor moving
plt.ion()
quadrotorView = plt.figure()
quadrotorArena = quadrotorView.add_subplot(111, projection='3d')

quadrotorArena.set_xlabel('X (meter)')
quadrotorArena.set_ylabel('Y (meter)')
quadrotorArena.set_zlabel('Z (meter)')

quadrotorArena.set_xlim(-5, 5)
quadrotorArena.set_ylim(-5, 5)
quadrotorArena.set_zlim(-2, 3)

drone1 = quadrotorArena.scatter(tello1X, tello1Y, tello1Z, color='red')
drone1BodyData = Tello1.getBodyPosition()
drone1Body = quadrotorArena.scatter(
    drone1BodyData[0], drone1BodyData[1], drone1BodyData[2], color='blue')
quadrotorView.show()

for i in range(200):
    print('-------Time:', Tello1.t, '-------')
    tello1State = Tello1.getState()
    tello2State = Tello2.getState()
    # [phi, theta, psi, zdot] (degree)
    Tello1.controlAttitude(
        [5*degreeToRadian, 5*degreeToRadian, 0*degreeToRadian, 5])
    Tello2.controlAttitude(
        [10*degreeToRadian, 0*degreeToRadian, 0*degreeToRadian, 5])

    # Tello limit ?
    Tello1.updateState()
    Tello2.updateState()

    # Plot Data
    tello1State = Tello1.getState()
    tello1BodyPosition = Tello1.getBodyPosition()
    Report1.updateReport(tello1State, Tello1.thrust, Tello1.moments)   
    Report2.updateReport(tello2State, Tello2.thrust, Tello2.moments)

    print('---------------------------------')

    # Quadrotor View
    plt.pause(0.1)

    drone1._offsets3d = ([tello1State[0][0]], [
                         tello1State[0][1]], [tello1State[0][2]])
    drone1Body._offsets3d = (
        tello1BodyPosition[0], tello1BodyPosition[1], tello1BodyPosition[2])

    plt.draw()

Report1.generateReport()
Report2.generateReport()
plt.pause(100)