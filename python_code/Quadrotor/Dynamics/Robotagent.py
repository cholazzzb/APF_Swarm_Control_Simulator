class Robotagent(object):
    def __init__(self, specs, initialState, initialInput, controllerPID):
        self.t = 0
        self.dt = 1

        self.mass = specs["mass"]
        
        # State
        self.Xposition = initialState[0]
        self.Yposition = initialState[1]
        
        # Control Variable
        self.thrust = initialInput
        self.max_thrust = 5
        self.min_thrust = -5

        # Controller
        self.Kp = controllerPID[0]
        self.Ki = controllerPID[1]
        self.Kd = controllerPID[2]

        self.x_err = 0
        self.x_err_prev = 0
        self.x_err_sum = 0
        self.x_err_dot = 0
        self.y_err = 0

    def getState(self):
        return [self.Xposition, self.Yposition, self.thrust, self.x_err, self.x_err_prev, self.x_err_sum, self.x_err_dot]

    def updateState(self):
        self.t = self.t + self.dt
        self.Xdot = self.mass * self.thrust
        self.Xposition = self.Xposition + self.Xdot * self.dt

    def control(self, target):
        self.x_err = target - self.Xposition
        self.x_err_dot = (self.x_err-self.x_err_prev)/self.dt
        self.thrust = self.Kp*self.x_err + self.Ki*self.x_err_sum  + self.Kd*self.x_err_dot
        if self.thrust>self.max_thrust:
            self.thrust = self.max_thrust
        if self.thrust<self.min_thrust:
            self.thrust = self.min_thrust
        self.x_err_prev = self.x_err
        self.x_err_sum = self.x_err_sum + self.x_err
