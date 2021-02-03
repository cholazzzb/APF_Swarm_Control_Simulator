import matplotlib.pyplot as plt

# Specificly for Quadrotor


class Simulator(object):
    def __init__(self, agent, arena):
        self.agent = agent
        self.arena = arena
        self.objectPositions = []
        self.objectBodies = []

    def initialDrawing(self, positionColor, rotorColor):
        self.position = self.arena.scatter([self.agent.position[0]], [
                                           self.agent.position[1]], [self.agent.position[2]], color=positionColor)
        bodyData = self.agent.getBodyPosition()
        self.body = self.arena.scatter(
            bodyData[0], bodyData[1], bodyData[2], color=rotorColor)

    def updateDrawing(self):
        bodyPosition = self.agent.getBodyPosition()

        plt.pause(0.1)
        self.position._offsets3d = ([self.agent.position[0]], [self.agent.position[1]], [self.agent.position[2]])
        self.body._offsets3d = (bodyPosition[0], bodyPosition[1], bodyPosition[2])
        plt.draw()
