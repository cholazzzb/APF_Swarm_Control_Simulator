import sys
sys.path.append('../')
from ObstaclePotentialField import ObstaclePotentialField
from Agent import Agent
from Obstacle import Obstacle

Drones = []
Drone1 = Agent(0, (1, 2, 0), 1)
Drones.append(Drone1)
Drone2 = Agent(1, (9, 9, 0), 1)
Drones.append(Drone2)

Obstacles = []
Obstacle1 = Obstacle((5,5,0))
Obstacles.append(Obstacle1)

OPF = ObstaclePotentialField(0.5, 1, 20)
OPF2 =  ObstaclePotentialField(0.5, 1, 100)

def test_calculate_obstacle_force():
    assert OPF.calculate_obstacle_force(0,0,Drones, Obstacles) == (1,1,1)

def test_calculate_obstacle_forces():
    assert OPF.calculate_obstacle_forces(Drones, Obstacle1) == []