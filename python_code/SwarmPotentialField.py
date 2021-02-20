import math
from tupleUtil import *


class SwarmPotentialField(object):
    def __init__(self, min_allowable_dist):
        self.positiveGain1 = 0.1
        self.positiveGain2 = 0.1
        self.positiveGain3 = 0.1
        self.min_allowable_dist = min_allowable_dist
        self.allowable_dist_range = 0.5
        self.damping_factor = 0.1
        self.swarm_force = (0, 0, 0)
    
    def setPositiveGain1(self, newPositiveGain1):
        self.positiveGain1 = newPositiveGain1
    
    def setPositiveGain2(self, newPositiveGain2):
        self.positiveGain2 = newPositiveGain2
    
    def setPositiveGain3(self, newPositiveGain3):
        self.positiveGain3 = newPositiveGain3

    def setDampingFactor(self, newDampingFactor):
        self.damping_factor = newDampingFactor

    def getDistance(self, agent1_index, agent2_index, Agents):
        distance_tuple = minusWithTuple(
            Agents[agent1_index].position, Agents[agent2_index].position)
        distance = math.sqrt(
            abs(sum(tuple(pow(x, 2) for x in distance_tuple))))
        return (distance_tuple, distance)

    def calculate_attractive_force(self, agent1_index, agent2_index, Agents):
        (distance_tuple, distance) = self.getDistance(agent1_index, agent2_index, Agents)
        print('distance between(', agent1_index, ',' , agent2_index, '): ', distance)

        if distance < self.min_allowable_dist:
            self.swarm_force = timesWithInteger(
                distance_tuple,
                ((1/distance - 1/self.min_allowable_dist)*self.positiveGain1/distance/distance
                    -
                 self.positiveGain2*(distance-self.min_allowable_dist))
                / distance
            )
        # Tambahan sendiri
        elif distance < self.min_allowable_dist + self.allowable_dist_range: 
            self.swarm_force = (0,0,0)
        else:
            self.swarm_force = timesWithInteger(
                distance_tuple, -self.positiveGain3*(distance-self.min_allowable_dist)/distance)

    def calculate_swarm_field_force(self, agent1_index, agent2_index, Agents):
        self.calculate_attractive_force(agent1_index, agent2_index, Agents)
        different_velocity = minusWithTuple(
            Agents[agent1_index].getVelocity(), Agents[agent2_index].getVelocity())
        damping_force = timesWithInteger(
            different_velocity, self.damping_factor)
        print('self.swarm_force', self.swarm_force)
        print('calculate_swarm_field_force', minusWithTuple(self.swarm_force, damping_force))
        return minusWithTuple(self.swarm_force, damping_force)

    def calculate_total_swarm_field_force(self, agent_index, Agents):
        total_swarm_field_force = (0, 0, 0)
        for Agent in Agents:
            if Agent.index == agent_index:
                pass
            else:
                total_swarm_field_force = plusWithTuple(
                    total_swarm_field_force, self.calculate_swarm_field_force(agent_index, Agent.index, Agents))
        return total_swarm_field_force
