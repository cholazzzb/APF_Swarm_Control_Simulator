import numpy as np
import matplotlib.pyplot as plt
import random
import math
# from numba import njit, jit, prange
import control as co

def calculateXDot(statesX, controlledVariablesU):
    x_dot = np.ravel(np.dot(A, statesX)) + np.ravel(np.dot(B, controlledVariablesU))
    return x_dot

def time_range(fraction, startTime, endTime):
    n = 1 + (endTime-startTime)*fraction
    timeArray = np.linspace(startTime, endTime, n)
    return timeArray

# min_ , max_ = min max value input (ex : rotor speed)
def PID_Control(k, min_, max_, timeArray):
    # Initialize the constants
    kp, ki, kd = k
    PID = np.zeros(u.shape)
    x0 = np.zeros((3, 1))
    P_ = 0
    I_ = 0
    D_ = 0
    err = 0
    e_sum = 0
    e_prev = 0

    x = np.empty((timeArray.shape[0], x0.shape[0]))
    y = np.empty((timeArray.shape[0], C.shape[0]))

    x[0] = np.ravel(x0)
    y[0] = np.ravel(np.dot(C, x[0])) + np.ravel(np.dot(D, u[0]))

    err = u[0] - y[0]
    e_sum = e_sum + err*(timeArray[1] - timeArray[0])

    # PID Loop
    for iteration in range(1, timeArray.shape[0]):
        # Euler method to find the output of the function
        dt = 0.01 #fraction = 100
        x[iteration] = x[iteration-1] + dt * \
            calculateXDot(x[iteration-1], PID[iteration-1])

        # Count the mean square error
        err = u[iteration] - y[iteration-1]

        # PID value set
        P_ = kp*err

        e_sum = e_sum + err*dt
        e_sum = np.fmax(np.fmin(e_sum, max_/ki), min_/ki)

        I_ = ki*e_sum

        D_ = kd*(err - e_prev)/dt

        e_prev = err

        # PID = PID in the graph?
        PID[iteration] = P_ + I_ + D_
        # Range for control output
        PID[iteration] = np.fmax(np.fmin(PID[iteration], max_), min_)
        
        y[iteration] = np.ravel(np.dot(C, x[iteration])) + \
            np.ravel(np.dot(D, PID[iteration]))

    return y, PID


def PIDcost(PIDParameter):
    y_, PID_ = PID_Control(PIDParameter, 0, 6, time_range(100, 0, 20)
                           )
    err = np.zeros(y_.shape[0])
    for iteration in range(y_.shape[0]):
        err[iteration] = u[iteration]-y_[iteration]
    return np.mean(abs(err))


def PSO(c_func, n_param, particles, lb, ub, iterate_max):
    # Define the constants
    w = 0.5
    c1 = 0.8
    c2 = 0.7
    iterate = 0

    # Setup the initial conditions for position and velocity arrays
    ppos_vector = np.random.uniform(lb, ub, (particles, n_param))
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


# Create the transfer function
sys = co.tf([2], [1, 3, 2, 1])

# Change from s-domain to state space
sys_ss = co.tf2ss(sys)

A = np.array(sys_ss.A)
B = np.array(sys_ss.B)
C = np.array(sys_ss.C)
D = np.array(sys_ss.D)
print("A", A)
print("B", B)
print("C", C)
print("D", D)


timeArray = time_range(100, 0, 20)
u = np.ones((timeArray.shape[0], 1))*3

swarm = PSO(PIDcost, 3, 5, 0, 100, 10)

# To check PID control result
y_1, PID_1 = PID_Control(swarm[1], 0, 6, timeArray)

plt.title("PID Control with PSO")
plt.xlabel("time (s)")
plt.ylabel("Output")
plt.plot(timeArray, y_1, label='Output')
plt.plot(timeArray, PID_1, label='PID Value', color='lightgray', linestyle='-.')
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
