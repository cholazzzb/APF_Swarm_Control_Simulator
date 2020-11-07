# Documentation

## Artificial Potential Field Concept

### Target Potential Field
- Force beetween Target and Agent
- constants:
1. damping factor
2. gain (to accelerate reaching to the target)
3. target detecting range

## Object
1. Agent
2. Target
3. TargetPotentialField


## Code Architecture
1. Each Object will return the valued calculated
2. The main.py will contain the state of variables to draw the graph


## Itung Manual
### Targel Potential Field
Drone 1 position = 0,0,0
Drone 1 mass = 1
Target 1 position = 9,9,9
gain = 1
target detecting range = 1
damping factor = 1

-- Iterasi 1 --
Attractive Force = -(1*(-9,-9,-9)/9akar3) = (0.57735026919, ..., ...)
Target Force = (0.57735026919, ..., ...) - 1 ((0,0,0)-(0,0,0)) = (0.57735026919, ..., ...)
New Velocity = (0.57735026919, ..., ...)
New Position = (0.57735026919, ..., ...)

-- Iterasi 2 --
Attractive Force = -(1*(-8.423,-8.423,-8.423)/-8.423akar3) = (0.57735026919, ..., ...)
Target Force = (0.57735026919, ..., ...) - 1 ((0.57735026919, ..., ...)-(0,0,0)) = (0, 0, 0)
New Velocity = (0.57735026919, ..., ...)
New Position = (1.15470053838, 1.15470053838, 1.15470053838)
