# Attitude Controller

dt = 1/15
totalData = 15 (phi), 12? ....

1. Phi
## Output
output = 8
outputReal = np.array([0.06349999999999997, -0.2453, -1.4906000000000001, -3.4156, -5.3574,
    -6.801599999999999, -7.716799999999999, -8.2524, -8.565000000000001,
    -8.701899999999998, -8.770700000000001, -8.7471, -8.613800000000001,
    -8.4334, -8.1632])
optimizeParameter = "phi"

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

## Result
Total iteration ->  100
Best fitness ->  5.358276432955151
Best parameter ->  [ 0.01  -0.001  0.007]

2. Theta
## Output
output = -8
outputReal = np.array([0.0258,
    -0.1076,
    -0.5204,
    -1.4209,
    -2.7322,
    -4.2668,
    -5.682,
    -6.801699999999999,
    -7.543100000000001,
    -7.8899,
    -7.9696,
    -7.918000000000001,
    -7.787899999999999,
    ])
optimizeParameter = "theta"

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

## Result
Total iteration ->  100
Best fitness ->  5.224040635303442
Best parameter ->  [ 0.022 -0.003  0.002]

3. Psi
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

## Result 
Total iteration ->  100
Best fitness ->  55.255738469129405
Best parameter ->  [0.001 0.002 0.002]

4. zdot
## Setup
## Time
dt = 1/15
startTime = 0
endTime = 5

simulationTime = np.linspace(
    startTime, endTime, 1 + int((endTime-startTime)/dt))

## Output
outputSetpoint = 0  # degree (angles), meter/second (velocity)
optimizeParameter = "zdot"

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

## Result
Total iteration ->  100
Best fitness ->  1.15212585356477
Best parameter ->  [ 7.86  3.87 -0.07]

# Position Controller
1. x

## Setup
## Time
dt = 1/3
startTime = 0
endTime = 5
Initial Position = 0,0,1 using attitude controller in 3data result

## Output
outputSetpoint = 1  # degree (angles), meter/second (velocity)
optimizeParameter = "x"

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

## Result
