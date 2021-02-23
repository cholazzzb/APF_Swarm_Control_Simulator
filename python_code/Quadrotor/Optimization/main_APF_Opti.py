import numpy as np
import matplotlib as plt
from myPSO import myPSO

dt = 0.01
startTime = 0
endTime = 5

simulationTime = np.linspace(
    startTime, endTime, 1 + int((endTime-startTime)/dt))

# Output
outputSetpoint = 1 # degree (angles), meter/second (velocity)

# PSO Parameter
w = 0.5
c1 = 0.3
c2 = 0.4
number_of_particles = 100
number_of_parameters = 4
min_param_value = 0
max_param_value = 1
total_iteration = 25
is_minimize = True
cost_function = "integralAbsoluteError"

PSO = myPSO(simulationTime, outputSetpoint, 0, 100, 'x')
PSO.particleSwarmOptimization(w, c1, c2,
                              number_of_particles, number_of_parameters, min_param_value, max_param_value, total_iteration, is_minimize, cost_function)
# PSO.bestParameter =  [93.8483854, 70.64057641, 23.2112922 ]

PSO.plotBestResponse()