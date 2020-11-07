from pylab import *
import math
from tupleUtil import *

# x = [0,2,-3,-1.5]
# y = [0,3,1,-2.5]
# color=['m','m','r','b']

# scatter(x,y, s=100 ,marker='o', c=color)

# show()


class TargetPotentialField(object):
    def __init__(self, damping_factor, gain, target_detecting_range):
        self.damping_factor = damping_factor  # matrix of tuples
        self.gain = gain  # matrix of tuples
        self.target_detecting_range = target_detecting_range  # vector of tuples
        self.attractive_force = (0,0,0)

    def calculate_attractive_force(self, agent_index, target_index, Agents, Targets):
        distance_tuple = minusWithTuple(
            Agents[agent_index].position, Targets[target_index].position)
        distance = sqrt(abs(sum(tuple(pow(x, 2) for x in distance_tuple))))
        if distance < self.target_detecting_range:
            self.attractive_force = timesWithInteger(distance_tuple, (-self.gain/self.target_detecting_range))
        else:
            self.attractive_force = timesWithInteger(distance_tuple, -self.gain/distance)

    def calculate_target_force(self, agent_index, target_index, Agents, Targets):
        self.calculate_attractive_force(agent_index, target_index, Agents, Targets)
        different_velocity = minusWithTuple(Agents[agent_index].getVelocity(), Targets[target_index].velocity)
        damping_force = timesWithInteger(different_velocity, self.damping_factor)
        return minusWithTuple(self.attractive_force, damping_force)
