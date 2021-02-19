import math
from TargetPotentialField import TargetPotentialField
from ObstaclePotentialField import ObstaclePotentialField
from SwarmPotentialField import SwarmPotentialField

class SwarmController(object):
    def __init__(self, TPFconfig, OPFconfig, SPFconfig):
        self.agents = []
        self.obstacles = []
        self.targets = []
        self.TPF = TargetPotentialField(TPFconfig["damping_factor"], TPFconfig["gain"], TPFconfig["target_detecting_range"])
        self.OPF = ObstaclePotentialField(OPFconfig["positiveGain1"], OPFconfig["positiveGain2"], OPFconfig["detecting_range"])
        self.SPF = SwarmPotentialField(SPFconfig["min_allowable_dist"])

    def addAgent(self, newAgent):
        self.agents.append(newAgent)

    def addObstacle(self, newObstacle):
        self.obstacles.append(newObstacle)

    def addTarget(self, newTarget):
        self.targets.append(newTarget)

    def calculateAgentsForces(self):
        for drone in self.agents:
            drone.TargetPotentialForce = self.TPF.calculate_target_force(drone.index, 0, self.agents, self.targets)
            drone.SwarmPotentialForce = self.SPF.calculate_total_swarm_field_force(drone.index, self.agents)
            drone.ObstaclePotentialForce = self.OPF.calculate_agent_obstacles_force(self.agents[drone.index], self.obstacles)