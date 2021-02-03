import math
from TargetPotentialField import TargetPotentialField
from ObstaclePotentialField import ObstaclePotentialField

class SwarmController(object):
    def __init__(self, TPFconfig, OPFconfig):
        self.agents = []
        self.obstacles = []
        self.targets = []
        self.TPF = TargetPotentialField(TPFconfig["damping_factor"], TPFconfig["gain"], TPFconfig["target_detecting_range"])
        self.OPF = ObstaclePotentialField(OPFconfig["positiveGain1"], OPFconfig["positiveGain2"], OPFconfig["detecting_range"])

    def addAgent(self, newAgent):
        self.agents.append(newAgent)

    def addObstacle(self, newObstacle):
        self.obstacles.append(newObstacle)

    def addTarget(self, newTarget):
        self.targets.append(newTarget)

    def calculateAgentsForces(self):
        for drone in self.agents:
            drone.TargetPotentialForce = self.TPF.calculate_target_force(drone.index, 0, self.agents, self.targets)