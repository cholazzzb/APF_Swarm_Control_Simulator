import numpy as np
import matplotlib.pyplot as plt
from myPSOPosition import myPSOPosition

# Setup
# Time
dt = 0.01
startTime = 0
endTime = 10    

simulationTime = np.linspace(
    startTime, endTime, 1 + int((endTime-startTime)/dt))

# Output
# target 1, 2, 3...
# outputSetopint = [[x1,y1,z1], [x2,y2,z2], ...] in (m)
outputSetpoint = [[1,0,1], [5,0,1], [5,5,1], [0,0,1], [0,0,0]]

# PSO Parameter
w = 0.72984
c1 = 0.72984 * 2.05
c2 = 0.72984 * 2.05
number_of_particles = 100
number_of_parameters = 15
min_param_value = 0
max_param_value = 1
total_iteration = 1000
is_minimize = True
cost_function = "integralAbsoluteError"

PSO = myPSOPosition(simulationTime, outputSetpoint, 'ga kepake kok haha', 'ga kepake kok haha')
PSO.particleSwarmOptimization(w, c1, c2,
                              number_of_particles, number_of_parameters, min_param_value, max_param_value, total_iteration, is_minimize, cost_function)
# PSO.bestParameter =  [93.8483854, 70.64057641, 23.2112922 ]

PSO.plotBestResponse()
