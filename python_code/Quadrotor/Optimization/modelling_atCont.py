import numpy as np
import matplotlib.pyplot as plt
from myPSOModelling import myPSOModelling

dt = 1/15 # Remember to change the dt in the model
totalData = 15
simulationTime = []

for iteration in range(totalData):
    simulationTime.append(dt*iteration)
simulationTimeNumpy = np.array(simulationTime)

## Output
output = 96
outputReal = np.array([38.24900000000001,
    38.844899999999996,
    40.5906,
    43.306000000000004,
    46.92959999999999,
    51.2216,
    55.9326,
    60.81699999999999,
    65.80709999999999,
    70.86099999999999,
    75.93730000000001,
    81.0249,
    86.1333,
    91.2939,
    96.5992
    ])
optimizeParameter = "psi"

## PSO Parameter
w = 0.72984
c1 = 0.72984 * 2.05
c2 = 0.72984 * 2.05

number_of_particles = 30
number_of_parameters = 3
min_param_value = 0
max_param_value = 10
total_iteration = 100
is_minimize = True
cost_function = "integralAbsoluteError"

PSO = myPSOModelling(simulationTimeNumpy, output, 'ga kepake kok haha', 'ga kepake kok haha', optimizeParameter, outputReal)
PSO.particleSwarmOptimization(w, c1, c2,
                              number_of_particles, number_of_parameters, min_param_value, max_param_value, total_iteration, is_minimize, cost_function)

PSO.plotBestResponse()
