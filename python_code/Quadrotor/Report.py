import matplotlib.pyplot as plt
import math

radianToDegree = 180/math.pi

# Specific for quadrotor
class Report(object):
    def __init__(self, Quadrotor):
        self.name = Quadrotor.name
        self.t = Quadrotor.t
        self.dt = Quadrotor.dt
        self.xLabel = [0]

        # Controlled Variables
        self.phi = [Quadrotor.angles[0]*radianToDegree]
        self.theta = [Quadrotor.angles[1]*radianToDegree]
        self.psi = [Quadrotor.angles[2]*radianToDegree]

        self.z_dot = [Quadrotor.position_dot[2]]

        # Position
        self.x = [Quadrotor.position[0]]
        self.y = [Quadrotor.position[1]]
        self.z = [Quadrotor.position[2]]

        # Velocity
        self.x_dot = [Quadrotor.position_dot[0]]
        self.y_dot = [Quadrotor.position_dot[1]]

        # Angular Velocity
        self.phi_dot = [Quadrotor.angles_dot[0]*radianToDegree]
        self.theta_dot = [Quadrotor.angles_dot[1]*radianToDegree]
        self.psi_dot = [Quadrotor.angles_dot[2]*radianToDegree]

        # Thrust and Moment
        self.thrust = [Quadrotor.thrust]
        self.moment1 = [Quadrotor.moments[0]]
        self.moment2 = [Quadrotor.moments[1]]
        self.moment3 = [Quadrotor.moments[2]]

    def updateReport(self, newStates, thrust, moments):
        self.t = self.t + self.dt
        self.xLabel.append(self.t)

        # Controlled Variables
        self.phi.append(newStates[2][0])
        self.theta.append(newStates[2][1])
        self.psi.append(newStates[2][2])

        self.z_dot.append(newStates[1][2])

        # Position
        self.x.append(newStates[0][0])
        self.y.append(newStates[0][1])
        self.z.append(newStates[0][2])

        # Velocity
        self.x_dot.append(newStates[1][0])
        self.y_dot.append(newStates[1][1])

        # Angular Velocity
        self.phi_dot.append(newStates[3][0])
        self.theta_dot.append(newStates[3][1])
        self.psi_dot.append(newStates[3][2])

        # Thrust and Moment
        self.thrust.append(thrust)
        self.moment1.append(moments[0])
        self.moment2.append(moments[1])
        self.moment3.append(moments[2])

    def generateReport(self):
        # Style
        plt.style.use('dark_background')

        # Controlled Variable
        report = plt.figure()
        plt.get_current_fig_manager().full_screen_toggle()

        plt.suptitle(self.name + " States - Thrust and Moment")
        
        plt.subplot(4, 4, 1)
        plt.ylabel('X (m)')
        plt.plot(self.xLabel, self.x)

        plt.subplot(4, 4, 2)
        plt.ylabel('Y (m)')
        plt.plot(self.xLabel, self.y)

        plt.subplot(4, 4, 3)
        plt.ylabel('Z (m)')
        plt.plot(self.xLabel, self.z)

        plt.subplot(4, 4, 5)
        plt.ylabel('X dot (m)')
        plt.plot(self.xLabel, self.x_dot)

        plt.subplot(4, 4, 6)
        plt.ylabel('Y dot (m)')
        plt.plot(self.xLabel, self.y_dot)

        plt.subplot(4, 4, 7)
        plt.ylabel('z dot(m/s)')
        plt.plot(self.xLabel, self.z_dot)

        plt.subplot(4, 4, 9)
        plt.ylabel('phi(degree)')
        plt.plot(self.xLabel, self.phi)

        plt.subplot(4, 4, 10)
        plt.ylabel('theta(degree)')
        plt.plot(self.xLabel, self.theta)

        plt.subplot(4, 4, 11)
        plt.ylabel('psi(degree)')
        plt.plot(self.xLabel, self.psi)

        plt.subplot(4, 4, 13)
        plt.ylabel('phi dot(degree)')
        plt.plot(self.xLabel, self.phi_dot)

        plt.subplot(4, 4, 14)
        plt.ylabel('theta dot(degree)')
        plt.plot(self.xLabel, self.theta_dot)

        plt.subplot(4, 4, 15)
        plt.ylabel('psi dot(degree)')
        plt.plot(self.xLabel, self.psi_dot)

        plt.subplot(4, 4, 4)
        plt.ylabel('thrust')
        plt.plot(self.xLabel, self.thrust)

        plt.subplot(4, 4, 8)
        plt.ylabel('moment1')
        plt.plot(self.xLabel, self.moment1)

        plt.subplot(4, 4, 12)
        plt.ylabel('moment2')
        plt.plot(self.xLabel, self.moment2)

        plt.subplot(4, 4, 16)
        plt.ylabel('moment3')
        plt.plot(self.xLabel, self.moment3)

        plt.show()
