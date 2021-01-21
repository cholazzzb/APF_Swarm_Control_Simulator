from pylab import *
import math
from tupleUtil import *


class SwarmPotentialField(object):
    def __init__(self, min_allowable_dist):
        self.positiveGain1 = 1
        self.positiveGain2 = 1
        self.positiveGain3 = 1
        self.min_allowable_dist = min_allowable_dist
        self.damping_factor = 1
        self.swarm_force = (0, 0, 0)

    def calculate_attractive_force(self, agent1_index, agent2_index, Agents):
        distance_tuple = minusWithTuple(
            Agents[agent1_index].position, Agents[agent2_index].position)
        distance = sqrt(abs(sum(tuple(pow(x, 2) for x in distance_tuple))))
        
        if distance < self.min_allowable_dist:
            self.swarm_force = timesWithInteger(
                distance_tuple,
                ((1/distance - 1/self.min_allowable_dist)*self.positiveGain1/distance/distance
                    -
                 self.positiveGain2*(distance-self.min_allowable_dist))
                 /distance
            )
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
        total_swarm_field_force = (0,0,0)
        for Agent in Agents:
            if Agent.index == agent_index:
                pass
            else:
                total_swarm_field_force = plusWithTuple(total_swarm_field_force, self.calculate_swarm_field_force(agent_index, Agent.index, Agents))
        return total_swarm_field_force
