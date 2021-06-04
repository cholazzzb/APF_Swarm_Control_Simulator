# Attitude Controller

dt = 1/15
totalData = 15 (phi), 13 (theta), 15 (psi)

initial and target z = 1

1. Phi
## Output
output = -8
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
Best fitness ->  5.296553433227793
Best parameter ->  [-1.022  0.623  0.049]

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
Best fitness ->  5.95170296701274
Best parameter ->  [-0.142 -0.135  0.028]

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
Best fitness ->  11.255567164385994
Best parameter ->  [-0.044 -0.031 -0.079]

4. zdot
## Setup
## Time
dt = 1/15
startTime = 0
endTime = 5

simulationTime = np.linspace(
    startTime, endTime, 1 + int((endTime-startTime)/dt))

## Output
outputSetpoint = 1  # degree (angles), meter/second (velocity)
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
Best fitness ->  0.0002476760612999396
Best parameter ->  [22.21 22.21  0.  ]

# Position Controller
1. z

## Setup
## Time
dt = 1/15
startTime = 0
endTime = 5

## Output
initial pos = 0,0,0
outputSetpoint = 1  # degree (angles), meter/second (velocity)
optimizeParameter = "z"

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
Best fitness ->  1.9469987887586866
Best parameter ->  [0.09 1.59 7.06]

2. x

3. y
