""" 
    File that contains all the neccessary configuration parameters for the 
    MPC Trajectory Generation Module
"""

class Config:
    def __init__(self):
        self.vehicle_width = 0.5
        self.vehicle_margin = 0.25
        self.N_hor = 10
        self.nu = 2
        self.nz = 10+9 # 9 costs
        self.nx = 3
        self.nobs = 3
        self.Nobs =  10
        self.ts = 0.2
        self.vmax = 1.5
        self.vmin = 0.0
        self.omega_max = 0.5
        self.omega_acc_max = 200
        self.acc_max = 1
        self.acc_min = -1
        self.q = 1
        self.qtheta = 2
        self.rv = 10
        self.rw = 10
        self.qN = 200
        self.qthetaN = 10
        self.qCTE = 1
        self.acc_penalty = 0.0
        self.omega_acc_penalty = 0.0
        self.num_steps_taken = 5
        self.throttle_ratio = 0.9
        self.build_directory = 'mpc_build'
        self.optimizer_name = 'navigation'
        self.bad_exit_codes = ["NotConvergedIterations", "NotConvergedOutOfTime"]
        