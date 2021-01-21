import sys
sys.path.append('../')

from Agent import Agent

Agent1 = Agent(0, (0, 0, 0), 10)
print('Velocity Before', Agent1.velocity)
Agent1.calculateVelocity((5, 5, 5))
print('Velocity After', Agent1.velocity)
