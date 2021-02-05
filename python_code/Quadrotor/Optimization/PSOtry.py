import numpy as np
import matplotlib.pyplot as plt
import random
import math
# from numba import njit, jit, prange
import sys
sys.path.append('../')
from Quadrotor import Quadrotor

# min_ , max_ = min max value input (ex : rotor speed)
def calculateYAndPID(k, min_, max_, timeArray, target):
    # Initialize the constants
    kp, ki, kd = k
    PID = np.zeros(target.shape)

    specs = {"mass": 1.25, "inertia": [
        0.0232, 0.0232, 0.0232], "armLength": 0.265}
    initialState = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    initialInput = [1.0, 0.0, 0.0, 0.0]
    attitudeControllerPID = [[kp, ki, kd],  # PID phi
                             [1.51433707, 0, 1.06242888],  # PID theta
                             [12.00891182, 0.0, 1.14173959],  # PID psi
                             [84.16079374, 0.08603913, 0.77345114]]  # PID z dot

    positionControllerPID = [[0, 0, 0],  # PID x
                             [37.79524657, 29.59992759, -0.10065972],  # PID y
                             [kp, ki, kd]]  # PID z

    model = Quadrotor(0, "model", specs, initialState,
                      initialInput, attitudeControllerPID, positionControllerPID)

    y = np.empty((timeArray.shape[0], 1))

    y[0] = model.position_dot[2]

    # PID Loop
    for iteration in range(1, timeArray.shape[0]):
        model.controlAttitude([10.0, 0.0,0.0,0.0])
        # model.controlPosition([0, 0, 10])
        model.updateState()

        y[iteration] = model.position_dot[2]
        PID[iteration] = model.KP_z + model.KI_z + model.KD_z

        if(model.position_dot[2] > 200 or model.position_dot[2] < -200):
            break
            return np.zeros((timeArray.shape[0], 1)), np.zeros((timeArray.shape[0], 1)), False

    return y, PID, True


def PIDcost(PIDParameter, target, timeArray):
    outputY, PID_, isPossible = calculateYAndPID(
        PIDParameter, 0, 6, timeArray, target)
    targetY = np.ones((timeArray.shape[0], 1))*10

    err = np.zeros(outputY.shape[0])
    if isPossible:
        for iteration in range(outputY.shape[0]):
            err[iteration] = targetY[iteration]-outputY[iteration]
        return np.mean(abs(err))
    else:
        return 9999


def PSO(isMinimize, cost_function, totalParameters, particles, min_param_value, max_param_value, total_iteration, target, timeArray):
    # Define the constants
    w = 0.9
    c1 = 0.3
    c2 = 0.4

    # Setup the initial conditions for position and velocity arrays
    if isMinimize:
        particle_fit_value = np.ones(particles) * 1e100
        global_fit_value = 1e100
    else:
        particle_fit_value = np.ones(particles) * 0
        global_fit_value = 0

    particle_position = np.random.uniform(
        min_param_value, max_param_value, (particles, totalParameters))
    particle_position_best = particle_position
    particle_velocity = np.zeros((particles, totalParameters))
    global_best_position = np.zeros(totalParameters)

    print('----------- PSO Algorithm starting ----------')

    plt.ion()
    window = plt.figure()
    animation = window.add_subplot(111, projection='3d')
    animation.set_xlabel("P")
    animation.set_ylabel("I")
    animation.set_zlabel("D")
    animation.set_xlim(min_param_value, max_param_value)
    animation.set_ylim(min_param_value, max_param_value)
    animation.set_zlim(min_param_value, max_param_value)

    animation_pos = animation.scatter(
        particle_position[:, 0], particle_position[:, 1], particle_position[:, 2], color="blue")

    window.show()
    # First loop
    for particle_index in range(particles):
        # Count
        cost_value = cost_function(
            particle_position[particle_index], target, timeArray)

        # Update
        if(particle_fit_value[particle_index] > cost_value):
            particle_fit_value[particle_index] = cost_value
            particle_position_best[particle_index] = np.copy(
                particle_position[particle_index])

        if(global_fit_value > cost_value):
            global_fit_value = cost_value
            global_best_position = np.copy(particle_position[particle_index])

    # Second loop
    iterate = 0
    while (iterate < total_iteration):
        for particle_index in range(particles):
            # Count
            particle_velocity[particle_index] = w*particle_velocity[particle_index] + c1*random.random()*(
                particle_position_best[particle_index]-particle_position[particle_index]) + c2*random.random()*(global_best_position-particle_position[particle_index])

            particle_position[particle_index] = particle_velocity[particle_index] + \
                particle_position[particle_index]

            cost_value = cost_function(
                particle_position[particle_index], target, timeArray)

            # Update
            if(particle_fit_value[particle_index] > cost_value):
                particle_fit_value[particle_index] = cost_value
                particle_position_best[particle_index] = np.copy(
                    particle_position[particle_index])

            if(global_fit_value > cost_value):
                global_fit_value = cost_value
                global_best_position = np.copy(
                    particle_position[particle_index])

        plt.pause(0.1)
        animation_pos._offsets3d = (
            particle_position[:, 0], particle_position[:, 1], particle_position[:, 2])
        plt.draw()

        iterate = iterate+1
        print("Iteration: ", iterate, " | Global best cost: ",
              global_fit_value, "PID :",  global_best_position)

    print("Best function", global_fit_value)
    print("Best position: ",
          global_best_position, " Iteration :  ", iterate)
    return global_best_position


fraction = 100
startTime = 0
endTime = 2

timeArray = np.linspace(startTime, endTime, 1 + (endTime-startTime)*fraction)

target = np.ones((timeArray.shape[0], 1))*10
min_param_value = 0
max_param_value = 100

# isMinimize, cost_function, totalParameters, particles, min_param_value, max_param_value, total_iteration, target
bestPID = PSO(True, PIDcost, 3, 100, min_param_value,
              max_param_value, 100, target, timeArray)

# To check PID control result
bestOutput, PID_input, isPossible = calculateYAndPID(
    bestPID, 0, 6, timeArray, target)

plt.title("PID Control with PSO")
plt.xlabel("time (s)")
plt.ylabel("Output")
plt.plot(timeArray, bestOutput, label='Output')
plt.plot(timeArray, PID_input, label='PID Value',
         color='lightgray', linestyle='-.')
plt.plot(timeArray, target, label='Setpoint', color='orange')
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
plt.show()
