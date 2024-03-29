import numpy as np
import matplotlib as plt
from myPSOAPF import myPSOAPF

dt = 0.01
startTime = 0
endTime = 10

simulationTime = np.linspace(
    startTime, endTime, 1 + int((endTime-startTime)/dt))

# Output
outputSetpoint = 1 # target Distance

# PSO Parameter
w = 0.72984
c1 = 0.72984 * 2.05
c2 = 0.72984 * 2.05
number_of_particles = 100
number_of_parameters = 8
min_param_value = 0
max_param_value = 150
total_iteration = 100
is_minimize = True
cost_function = "objectiveFunction"

# Cost Function 
# J = Alpha*Js + Beta*Jd

PSO = myPSOAPF(simulationTime, outputSetpoint, 0, 100, 'x')
PSO.setCostFunctionWeight(1, 1, 1)
PSO.particleSwarmOptimization(w, c1, c2,
                              number_of_particles, number_of_parameters, min_param_value, max_param_value, total_iteration, is_minimize, cost_function)
# PSO.bestParameter =  [93.8483854, 70.64057641, 23.2112922 ]

PSO.plotBestResponse()