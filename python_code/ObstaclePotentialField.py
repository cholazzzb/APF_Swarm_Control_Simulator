from pylab import *
import math
from tupleUtil import *


class ObstaclePotentialField(object):
    def __init__(self, positiveGain1, positiveGain2, detecting_range):
        self.positiveGain1 = positiveGain1
        self.positiveGain2 = positiveGain2
        self.detecting_range = detecting_range

    def calculate_obstacle_force(self, agent_index, obstacle_index, Agents, Obstacles):
        distance_tuple = minusWithTuple(
            Agents[agent_index].position, Obstacles[obstacle_index].position)
        distance = sqrt(abs(sum(tuple(pow(x, 2) for x in distance_tuple))))
        if distance <= self.detecting_range:
            return timesWithInteger(distance_tuple, (
                (1/distance - 1/self.detecting_range) *
                self.positiveGain1/distance/distance
                -
                self.positiveGain2*(distance-self.detecting_range)
            )/distance)
        else:
            return (0, 0, 0)
    
    def calculate_obstacle_forces(self, Agents, Obstacle):
        obstacles_forces = []
        for Agent in Agents:
            distance_tuple = minusWithTuple(
            Agent.position, Obstacle.position)
            distance = sqrt(abs(sum(tuple(pow(x, 2) for x in distance_tuple))))
            if distance <= self.detecting_range:
                obstacles_forces.append(timesWithInteger(distance_tuple, (
                    (1/distance - 1/self.detecting_range) *
                    self.positiveGain1/distance/distance
                    -
                    self.positiveGain2*(distance-self.detecting_range)
                )/distance))
            else:
                obstacles_forces.append(timesWithInteger(Agent.ObstaclePotentialForce, -1))
        return obstacles_forces
    

