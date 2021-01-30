import matplotlib.pyplot as plt

class AgentReport(object):
    def __init__(self, Agent):      
        self.t = Agent.t
        self.dt = Agent.dt
        self.xLabel = [0]

        self.xPosition = [Agent.Xposition]
        self.yPosition = [Agent.Yposition]

        self.thrust = [Agent.thrust]
        self.x_err = [Agent.x_err]
        self.x_err_prev = [Agent.x_err]
        self.x_err_sum = [Agent.x_err_sum]
        self.x_err_dot = [Agent.x_err_dot]

    def updateReport(self, newStates):
        self.t = self.t + self.dt
        self.xLabel.append(self.t)

        self.xPosition.append(newStates[0])
        self.yPosition.append(newStates[1])
        self.thrust.append(newStates[2])
        self.x_err.append(newStates[3])
        self.x_err_prev.append(newStates[4])
        self.x_err_sum.append(newStates[5])
        self.x_err_dot.append(newStates[6])

    def generateReport(self):
        plt.style.use('dark_background')
        plt.figure()
        plt.get_current_fig_manager().full_screen_toggle()
        plt.suptitle("States")
        
        plt.subplot(4,2,1)
        plt.ylabel('X')
        plt.plot(self.xLabel, self.xPosition)

        plt.subplot(4,2,2)
        plt.ylabel('Y')
        plt.plot(self.xLabel, self.yPosition)

        plt.subplot(4,2,3)
        plt.ylabel('X err')
        plt.plot(self.xLabel, self.x_err)

        plt.subplot(4,2,4)
        plt.ylabel('X err prev')
        plt.plot(self.xLabel, self.x_err_prev)

        plt.subplot(4,2,5)
        plt.ylabel('X err sum')
        plt.plot(self.xLabel, self.x_err_sum)

        plt.subplot(4,2,6)
        plt.ylabel('X err dot')
        plt.plot(self.xLabel, self.x_err_dot)

        plt.subplot(4,2,7)
        plt.ylabel('Thrust')
        plt.plot(self.xLabel, self.thrust)