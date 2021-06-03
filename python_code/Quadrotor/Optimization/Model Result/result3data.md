# Attitude Controller

dt = 0.33
totalData = 3

1. Phi
# Output
output = -9
outputReal = np.array([-1.9162, -9.4504, -9.2024])
optimizeParameter = "phi"

# PSO Parameter
w = 0.72984
c1 = 0.72984 * 2.05
c2 = 0.72984 * 2.05

number_of_particles = 30
number_of_parameters = 3
min_param_value = 0
max_param_value = 100
total_iteration = 100
is_minimize = True
cost_function = "integralAbsoluteError"

# Result
Total iteration ->  100
Best fitness ->  1.9175333333333349
Best parameter ->  [ 2.635e+01 -5.544e+01  2.000e-02]

2. Theta

# Output
output = -9
outputReal = np.array([-2.6364000000000005, -9.1562, -8.0896])
optimizeParameter = "theta"

# PSO Parameter
w = 0.72984
c1 = 0.72984 * 2.05
c2 = 0.72984 * 2.05

number_of_particles = 30
number_of_parameters = 3
min_param_value = 0
max_param_value = 100
total_iteration = 100
is_minimize = True
cost_function = "integralAbsoluteError"

# Result
Total iteration ->  100
Best fitness ->  3.1950827586206896
Best parameter ->  [-13.7  -22.83   0.45]

3. Psi
# Output
output = -9
outputReal = np.array([23.764400000000002, 45.3352, 73.184])
optimizeParameter = "psi"

# PSO Parameter
w = 0.72984
c1 = 0.72984 * 2.05
c2 = 0.72984 * 2.05

number_of_particles = 30
number_of_parameters = 3
min_param_value = 0
max_param_value = 100
total_iteration = 100
is_minimize = True
cost_function = "integralAbsoluteError"

# Result
Total iteration ->  100
Best fitness ->  33.95076981132073
Best parameter ->  [ -6.99 -10.    -2.  ]

