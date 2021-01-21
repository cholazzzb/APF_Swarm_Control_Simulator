import sys
sys.path.append('../')
from Quadrotor import Quadrotor

specs = {"mass": 1.25, "inertia": [0.0232, 0.0232, 0.0468]}
initialState = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
initialInput = [0, 0, 0, 0]
attitudeControllerPID = [[0.2, 0.0, 0.15],
                         [0.2, 0.0, 0.15],
                         [0.8, 0.0, 0.3],
                         [10.0, 0.2, 0.0]]

Tello1 = Quadrotor(specs, initialState, initialInput, attitudeControllerPID)
for i in range(100):
    print('-------Time:', Tello1.t, '-------')
    print(Tello1.getState())
    Tello1.controlAttitude([10,10,10,10])
    Tello1.updateState()
    print(Tello1.getState())
    print('---------------------------------')
