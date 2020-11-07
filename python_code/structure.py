import numpy as np

# Configuration
number_of_agent = 4
number_of_obstacles = 5
number_of_goals = 9
iteration = 0 # iteration equals to time

# obstacles random position generator = ORPG
auto_ORPG_is_on = False
# goals random position generator = GRPG
auto_GRPG_is_on = False

# setup obstacles position manual

# setup goals position manual


# Parameter
## Target Potential Field
damping_factor = []
gain = []
target_detecting_range = []

# Setup
agent_position = (0, 0, 0)
agents_position_ = []

for i in range(0, number_of_agent):
    x, y, z = agent_position
    agent_position = (x+1, y, z)
    agents_position.append(agent_position)

print(agents_position)

# Simulation
## Swarm Algorithm
### Artificial Potential Field

#### Obstacle Potential Field


#### Swarm Potential Field

#### Target Potential Field


# Testing

# A = np.array([[1,2,3], [2,3,4], [1,2,3]])
# B = np.array([[2,3,4], [1,2,3], [2,3,4]])
# C = np.multiply(A, B)
# D = np.divide(A, B)
# print(5*A)

