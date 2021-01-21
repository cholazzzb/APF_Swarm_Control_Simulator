import sys
sys.path.append('../')
from SwarmPotentialField import SwarmPotentialField
from Agent import Agent

Drones = []
Drone1 = Agent(0, (1, 2, 0), 1)
Drones.append(Drone1)
Drone2 = Agent(1, (1.5, 2, 0), 1)
Drones.append(Drone2)
Drone3 = Agent(2, (11.5, 2, 0), 1)
Drones.append(Drone3)

SPF = SwarmPotentialField(1)

def test_calculate_swarm_force():
    assert SPF.calculate_swarm_field_force(0,1,Drones) == (-2.25,0,0)

def test_calculate_total_swarm_field_force():
    assert SPF.calculate_total_swarm_field_force(Drone1.index, Drones) == (0,0,0)

def test_calculate_total_swarm_field_force2():
    assert SPF.calculate_total_swarm_field_force(Drone2.index, Drones) == (0,0,0)