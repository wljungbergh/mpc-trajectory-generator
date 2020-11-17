""" 
    File that contains all the neccessary configuration parameters for the 
    MPC Trajectory Generation Module
"""

class Config:
    def __init__(self):
        self.vehicle_width = 0.5
        self.vehicle_margin = 0.25
        self.N_hor = 20
        self.nu = 2
        self.nz = 10
        self.nx = 3
        self.nobs = 3
        self.Nobs =  10
        self.ts = 0.2
        self.vmax = 1.5
        self.omega_max = 0.5
        self.omega_acc_max = 2
        self.acc_max = 0.2
        self.acc_min = -0.2
        self.q = 0.01
        self.qtheta = 0
        self.rv = 0
        self.rw = 0
        self.qN = 0.1
        self.qthetaN = 0
        self.qCTE = 0.5
        self.acc_penalty = 0.5
        self.omega_acc_penalty = 0.5
        self.num_steps_taken = 10
        self.throttle_ratio = 0.9
        self.build_directory = 'mpc_build'
        self.optimizer_name = 'navigation'
        self.bad_exit_codes = ["NotConvergedIterations", "NotConvergedOutOfTime"]
        