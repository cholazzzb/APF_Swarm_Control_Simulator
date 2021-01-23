import sys
sys.path.append('../')
from Quadrotor import Quadrotor
import matplotlib.pyplot as plt
import math

radianToDegree = 180/math.pi
degreeToRadian = math.pi/180

specs = {"mass": 1.25, "inertia": [0.0232, 0.0232, 0.0232], "armLength": 0.265}
initialState = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
initialInput = [0.0, 0.0, 0.0, 0.0]
attitudeControllerPID = [[0.2, 0.0, 0.15],  # PID phi
                         [0.2, 0.0, 0.15],  # PID theta
                         [0.8, 0.0, 0.3],  # PID psi
                         [10.0, 0.2, 0.0]]  # PID z dot

Tello1 = Quadrotor(specs, initialState, initialInput, attitudeControllerPID)
xLabel = [Tello1.t]
tello1State = Tello1.getState()
tello1BodyPosition = Tello1.getBodyPosition()

# Controlled variable
tello1ZDot = [tello1State[1][2]]
tello1phi = [tello1State[2][0]]
tello1theta = [tello1State[2][1]]
tello1psi = [tello1State[2][2]]

# Position
tello1X = [tello1State[0][0]]
tello1Y = [tello1State[0][1]]
tello1Z = [tello1State[0][2]]


# Testing purpose
tello1XDot = [tello1State[1][0]]
tello1YDot = [tello1State[1][1]]

tello1phiDot = [tello1State[3][0]]
tello1thetaDot = [tello1State[3][1]]
tello1psiDot = [tello1State[3][2]]

tello1torque = [Tello1.torque]
tello1moment1 = [Tello1.moment[0]]
tello1moment2 = [Tello1.moment[1]]
tello1moment3 = [Tello1.moment[2]]

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
    # [phi, theta, psi, zdot] (degree)
    Tello1.controlAttitude(
        [5*degreeToRadian, 5*degreeToRadian, 0*degreeToRadian, 5])
    # Tello limit ?
    Tello1.updateState()

    # Plot Data
    tello1State = Tello1.getState()
    tello1BodyPosition = Tello1.getBodyPosition()
    xLabel.append(Tello1.t)

    # Controlled Variable
    tello1ZDot.append(tello1State[1][2])
    tello1phi.append(tello1State[2][0]*radianToDegree)
    tello1theta.append(tello1State[2][1]*radianToDegree)
    tello1psi.append(tello1State[2][2]*radianToDegree)

    # Position
    tello1X.append(tello1State[0][0])
    tello1Y.append(tello1State[0][1])
    tello1Z.append(tello1State[0][2])

    # Testing Purpose
    tello1XDot.append(tello1State[1][0])
    tello1YDot.append(tello1State[1][1])

    tello1phiDot.append(tello1State[3][0]*radianToDegree)
    tello1thetaDot.append(tello1State[3][1]*radianToDegree)
    tello1psiDot.append(tello1State[3][2]*radianToDegree)

    tello1torque.append(Tello1.torque)
    tello1moment1.append(Tello1.moment[0])
    tello1moment2.append(Tello1.moment[1])
    tello1moment3.append(Tello1.moment[2])

    print('---------------------------------')

    # Quadrotor View
    plt.pause(0.1)

    drone1._offsets3d = ([tello1State[0][0]], [
                         tello1State[0][1]], [tello1State[0][2]])
    drone1Body._offsets3d = (
        tello1BodyPosition[0], tello1BodyPosition[1], tello1BodyPosition[2])

    plt.draw()

# Plot
plt.style.use('dark_background')

# Controlled Variable
plt.figure()
plt.get_current_fig_manager().full_screen_toggle()

plt.suptitle("Controlled Variables")
plt.subplot(4, 1, 1)
plt.ylabel('phi(degree)')
plt.plot(xLabel, tello1phi)

plt.subplot(4, 1, 2)
plt.ylabel('theta(degree)')
plt.plot(xLabel, tello1theta)

plt.subplot(4, 1, 3)
plt.ylabel('psi(degree)')
plt.plot(xLabel, tello1psi)

plt.subplot(4, 1, 4)
plt.ylabel('zdot(m/s)')
plt.plot(xLabel, tello1ZDot)


# Position
plt.figure()
plt.get_current_fig_manager().full_screen_toggle()

plt.suptitle("Tello1 Position")

plt.subplot(3, 1, 1)
plt.ylabel('X(m)')
plt.plot(xLabel, tello1X)

plt.subplot(3, 1, 2)
plt.ylabel('Y(m)')
plt.plot(xLabel, tello1Y)

plt.subplot(3, 1, 3)
plt.ylabel('Z(m)')
plt.plot(xLabel, tello1Z)

# Testing

plt.figure()
plt.get_current_fig_manager().full_screen_toggle()

plt.suptitle("Tello1 Velocity")

plt.subplot(2, 1, 1)
plt.ylabel('X dot (m)')
plt.plot(xLabel, tello1XDot)

plt.subplot(2, 1, 2)
plt.ylabel('Y dot (m)')
plt.plot(xLabel, tello1YDot)

plt.figure()
plt.get_current_fig_manager().full_screen_toggle()

plt.suptitle("Tello1 Anglular Velocity")

plt.subplot(3, 1, 1)
plt.ylabel('phi dot(degree)')
plt.plot(xLabel, tello1phiDot)

plt.subplot(3, 1, 2)
plt.ylabel('theta dot(degree)')
plt.plot(xLabel, tello1thetaDot)

plt.subplot(3, 1, 3)
plt.ylabel('psi dot(degree)')
plt.plot(xLabel, tello1psiDot)


plt.figure()
plt.get_current_fig_manager().full_screen_toggle()
plt.suptitle("Tello1 Moment")

plt.subplot(4, 1, 1)
plt.ylabel('torque')
plt.plot(xLabel, tello1torque)

plt.subplot(4, 1, 2)
plt.ylabel('moment1')
plt.plot(xLabel, tello1moment1)

plt.subplot(4, 1, 3)
plt.ylabel('moment2')
plt.plot(xLabel, tello1moment2)

plt.subplot(4, 1, 4)
plt.ylabel('moment3')
plt.plot(xLabel, tello1moment3)

plt.show()
plt.pause(100)
