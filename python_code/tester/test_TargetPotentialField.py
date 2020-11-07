import sys
sys.path.append('../')
from TargetPotentialField import TargetPotentialField
from Agent import Agent
from Target import Target

Drones = []
Drone1 = Agent((1, 2, 0))
Drones.append(Drone1)
Drone2 = Agent((2, 2, 0))
Drones.append(Drone2)

Targets = []
Ship1 = Target((5,5,0))
Targets.append(Ship1)

TPF = TargetPotentialField(0.5, 1, 1)
TPF2 =  TargetPotentialField(0.5, 1, 100)

def test_calculate_attractive_force():
    assert TPF.calculate_attractive_force(0, 0, Drones, Targets) == (0.8, 0.6000000000000001, -0.0)

def test_calculate_attractive_force2():
    assert TPF2.calculate_attractive_force(0, 0, Drones, Targets) == (0.04, 0.03, -0.0)
