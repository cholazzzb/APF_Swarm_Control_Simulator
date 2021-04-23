import math
import numpy as np

degreeToRadian = math.pi/180
radianToDegree = 180/math.pi

# Rotation Matrix -> QTGM (Quadrotor To Global Matrix) Only from Yaw
def QTGMRotationMatrix(psi):
    return np.array([
            [math.cos(psi), -math.sin(psi), 0],
            [math.sin(psi), math.cos(psi), 0],
            [0, 0, 1]])

def QTGM(angles, vector):
    psi = angles[2]
    rotationMatrix = QTGMRotationMatrix(psi)

    return rotationMatrix.dot(vector)

def GTQM(angles, vector):
    psi = angles[2]
    rotationMatrix = QTGMRotationMatrix(psi).T

    return rotationMatrix.dot(vector)

angles = (33.66*degreeToRadian, -0.11*degreeToRadian, -90*degreeToRadian)

vector = [0, -1, .95]
print(QTGM(angles, vector))

print(GTQM(angles, [-1, 0, 0]))