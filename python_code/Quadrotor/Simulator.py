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
        bodyPosition = self.agent.getBodyPosition()
        frontRotor = bodyPosition[0]
        otherRotors = bodyPosition[1]

        self.frontRotor = self.arena.scatter(
            frontRotor[0], frontRotor[1], frontRotor[2], color=positionColor)

        self.otherRotors = self.arena.scatter(
            otherRotors[0], otherRotors[1], otherRotors[2], color=rotorColor)
    

    def updateDrawing(self):
        bodyPosition = self.agent.getBodyPosition()
        frontRotor = bodyPosition[0]
        otherRotors = bodyPosition[1]
        
        plt.pause(0.01)
        self.position._offsets3d = ([self.agent.position[0]], [self.agent.position[1]], [self.agent.position[2]])
        self.frontRotor._offsets3d = ([frontRotor[0]], [frontRotor[1]], [frontRotor[2]])
        self.otherRotors._offsets3d = (otherRotors[0], otherRotors[1], otherRotors[2])
        plt.draw()
