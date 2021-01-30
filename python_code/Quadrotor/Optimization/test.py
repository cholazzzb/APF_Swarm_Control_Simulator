from myPSO import myPSO
import sys
sys.path.append('../')
from Quadrotor import Quadrotor

def cost_function():
    timeArray = time_range(100, 0, 20)
    outputY, PID_ = calculateYAndPID(PIDParameter, 0, 6, timeArray, target)
    targetY = np.ones((timeArray.shape[0], 1))*3

    err = np.zeros(outputY.shape[0])
    for iteration in range(outputY.shape[0]):
        err[iteration] = targetY[iteration]-outputY[iteration]
    return np.mean(abs(err))

PSO = myPSO(100, Quadrotor, cost_function, 2)
bestPID = PSO.calculateBestParameter()

# calculate bestOutput within model

# Plot