import numpy as np
import matplotlib.pyplot as plt
import random
import math
# from numba import njit, jit, prange
import control as co

import sys
sys.path.append('../')
from Quadrotor import Quadrotor
from Report import Report
from Simulator import Simulator

radianToDegree = 180/math.pi
degreeToRadian = math.pi/180

specs = {"mass": 1.25, "inertia": [0.0232, 0.0232, 0.0232], "armLength": 0.265}
initialState = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]

initialInput = [0.0, 0.0, 0.0, 0.0]


def calculateXDot(statesX, controlledVariablesU):
    x_dot = np.ravel(np.dot(A, statesX)) + np.ravel(np.dot(B, controlledVariablesU))
    return x_dot

def time_range(fraction, startTime, endTime):
    n = 1 + (endTime-startTime)*fraction
    timeArray = np.linspace(startTime, endTime, n)
    return timeArray

# min_ , max_ = min max value input (ex : rotor speed)
def calculateYAndPID(k, min_, max_, timeArray):

    kp, ki, kd = k
    attitudeControllerPID = [[kp, ki, kd],  # PID phi
                         [0.0, 0.0, 0.0],  # PID theta
                         [0.0, 0.0, 0.0],  # PID psi
                         [0.0, 0.0, 0.0]]  # PID z dot

    positionControllerPID = [[5.2, 0.0, 5.15], # PID x
                             [5.2, 0.0, 5.15], # PID y
                             [0.8, 0.0, 0.3]] # PID z

    Tello1 = Quadrotor("Tello1", specs, initialState, initialInput, attitudeControllerPID, positionControllerPID)
    Report1 = Report(Tello1)
    # Initialize the constants
    PID = np.zeros(timeArray.shape)
    x0 = np.ravel(np.array(Tello1.getState())) #np.zeros((12, 1))
    # P_ = 0
    # I_ = 0
    # D_ = 0
    err = 0
    e_sum = 0
    e_prev = 0

    x = np.empty((timeArray.shape[0], x0.shape[0]))
    y = np.empty((timeArray.shape[0], 1))

    x[0] = np.ravel(x0)
    #z_dot
    y[0] = np.ravel(x0[5])

    err = u[0] - y[0]
    e_sum = e_sum + err*(timeArray[1] - timeArray[0])
    isStable = True

    # PID Loop
    for iteration in range(1, timeArray.shape[0]):
        Tello1.controlAttitude([3.0*degreeToRadian, 0.0, 0.0, 0.0])
        Tello1.updateState()
        Report1.updateReport(Tello1.getState(), Tello1.thrust, Tello1.moments)   

        x[iteration] = np.ravel(np.array(Tello1.getState()))
        # Euler method to find the output of the function
        dt = 0.01 #fraction = 100
        # x[iteration] = x[iteration-1] + dt * \
        #     calculateXDot(x[iteration-1], PID[iteration-1])

        # Count the mean square error
        # err = u[iteration] - y[iteration-1]

        # PID value set
        # P_ = kp*err

        e_sum = e_sum + err*dt
        e_sum = np.fmax(np.fmin(e_sum, max_/ki), min_/ki)

        # I_ = ki*e_sum

        # D_ = kd*(err - e_prev)/dt

        # e_prev = err

        # PID = PID in the graph?
        #PID[iteration] = P_ + I_ + D_
        # Range for control output
        #PID[iteration] = np.fmax(np.fmin(PID[iteration], max_), min_)
        
        # y[iteration] = np.ravel(np.dot(C, x[iteration])) + \
        #     np.ravel(np.dot(D, PID[iteration]))
        y[iteration] = x[iteration][6] # 5 = z_dot, phi = 6, theta = 7, psi = 8
        if(y[iteration] > 10000 or y[iteration]<-10000):
            print('masa sih', y[iteration])
            return y, False
    # Report1.generateReport()

    return y, isStable #, PID


def PIDcost(PIDParameter):
    timeArray = time_range(100, 0, 20)
    outputY, isStable = calculateYAndPID(PIDParameter, 0, 6, timeArray)
    if(isStable):
        err = np.zeros(outputY.shape[0])
        for iteration in range(outputY.shape[0]):
            err[iteration] = u[iteration]-outputY[iteration]
        return np.mean(abs(err))
    else:
        return 999

def PSO(c_func, n_param, particles, lowBoundary, upperBoundary, iterate_max):
    # Define the constants
    w = 0.5
    c1 = 0.8
    c2 = 0.7
    iterate = 0

    # Setup the initial conditions for position and velocity arrays
    ppos_vector = np.random.uniform(lowBoundary, upperBoundary, (particles, n_param))
    print('ppos_vector', ppos_vector)
    pbest_pos = ppos_vector
    pfit_value = np.ones(particles) * 1e100
    gbest_pos = np.zeros(n_param)
    gfit_value = 1e100
    pvel_vector = np.zeros((particles, n_param))

    # First loop for assigning the fitness value using the cost function
    for particle_number in range(particles):
        # Check the position of individual and group value using the evaluation function
        cost_func = c_func(ppos_vector[particle_number])

        # Update each values using the cost functions
        if(pfit_value[particle_number] > cost_func):
            pfit_value[particle_number] = cost_func
            pbest_pos[particle_number] = np.copy(ppos_vector[particle_number])

        if(gfit_value > cost_func):
            gfit_value = cost_func
            gbest_pos = np.copy(ppos_vector[particle_number])

    # Second loop for implementing the PSO Algorithm
    while (iterate < iterate_max):
        for particle_number in range(particles):
            # Update the velocity and position vector
            pvel_vector[particle_number] = w*pvel_vector[particle_number] + c1*random.random()*(
                pbest_pos[particle_number]-ppos_vector[particle_number]) + c2*random.random()*(gbest_pos-ppos_vector[particle_number])
            ppos_vector[particle_number] = pvel_vector[particle_number] + \
                ppos_vector[particle_number]

            cost_func = c_func(ppos_vector[particle_number])

            # Update each values using the cost functions
            if(pfit_value[particle_number] > cost_func):
                pfit_value[particle_number] = cost_func
                pbest_pos[particle_number] = np.copy(
                    ppos_vector[particle_number])

            if(gfit_value > cost_func):
                gfit_value = cost_func
                gbest_pos = np.copy(ppos_vector[particle_number])

        iterate = iterate+1
        print("Iteration: ", iterate, " | Global best cost: ", c_func(gbest_pos))

    print(c_func(gbest_pos))
    print("The best position for each parameter: ",
          gbest_pos, " with ", iterate, " iteration.")
    return ppos_vector, gbest_pos  # personal vector position, global best position

timeArray = time_range(1, 0, 1) # fractino , startTime, endTime
u = np.ones((timeArray.shape[0], 1))*3

swarm = PSO(PIDcost, 3, 1, 0, 100, 2) #cost function, totalParameter, particels, lowBoundary, higherBoundary, number of iteration

# To check PID control result
y_1= calculateYAndPID(swarm[1], 0, 6, timeArray)
# y_1= calculateYAndPID([10.0, 0.2, 0.0], 0, 6, timeArray)

plt.title("PID Control with PSO")
plt.xlabel("time (s)")
plt.ylabel("Output")
plt.plot(timeArray, y_1, label='Output')
# plt.plot(timeArray, PID_1, label='PID Value', color='lightgray', linestyle='-.')
plt.plot(timeArray, u, label='Setpoint', color='orange')
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
plt.show()


'''
40 Particles
Iteration:  100  | Global best cost:  0.20032024337131488
0.20032024337131488
The best position for each parameter:  [ 21.36521058 101.08983129  34.38245106]  with  100  iteration.

25 Particles
Iteration:  100  | Global best cost:  0.2019538228354657
0.2019538228354657
The best position for each parameter:  [14.74262487 69.83756253 23.78618052]  with  100  iteration.

5 Particles
Iteration:  100  | Global best cost:  0.21207816586769448
0.21207816586769448
The best position for each parameter:  [40.82775409 64.10530956 62.47613214]  with  100  iteration.

5 Particles
0.20205683234949473
The best position for each parameter:  [14.01403908 69.66350909 22.59364158]  with  10  iteration.

'''
