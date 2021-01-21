class AttitudeController(object):
    def __init__(self, PID_phi, PID_theta, PID_psi, dt):
        self.dt = dt

        self.KP_phi = PID_phi[0]
        self.KI_phi = PID_phi[1]
        self.KD_phi = PID_phi[2]

        self.KP_theta = PID_theta[0]
        self.KI_theta = PID_theta[1]
        self.KD_theta = PID_theta[2]

        self.KP_psi = PID_psi[0]
        self.KI_psi = PID_psi[1]
        self.KD_psi = PID_psi[2]

        self.phi_err = 0
        self.phi_err_prev = 0
        self.phi_err_sum = 0

        self.theta_err = 0
        self.theta_err_prev = 0
        self.theta_err_sum = 0

        self.psi_err = 0
        self.psi_err_prev = 0
        self.psi_err_sum = 0

    def control(self, target, current):
        self.phi_err = target[0] - current[0]
        self.theta_err = target[1] - current[1]
        self.psi_err = target[2] - current[2]

        # With PID
        u2 = self.KPphi * phi_err + self.KI_phi*self.phi_err_sum + \
            self.KDphi * (self.phi_err - self.phi_err_prev)/self.dt

        u3 =self.KPtheta * theta_err + self.KI_theta*self.theta_err_sum + \
            self.KDtheta * (self.theta_err - self.theta_err_prev)/self.dt

        u4 =self.KPpsi * psi_err + self.KI_psi*self.psi_err_sum + \
            self.KDpsi * (self.psi_err - self.psi_err_prev)/self.dt

        self.phi_err_prev = self.phi_err
        self.phi_err_sum = self.phi_err_sum + self.phi_err
        self.theta_err_prev = self.theta_err 
        self.theta_err_sum = self.theta_err_sum + self.theta_err
        self.psi_err_prev = self.psi_err
        self.psi_err_sum = self.psi_err_sum + self.psi_err