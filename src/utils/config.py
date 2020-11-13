""" 
    File that contains all the neccessary configuration parameters for the 
    MPC Trajectory Generation Module
"""

class Config:
    def __init__(self):
        self.vehicle_width = 0.5
        self.vehicle_margin = 0.1
        self.N_hor = 15                    
        self.nu = 2                          
        self.nz = 6                           
        self.nx = 3
        self.nobs = 3                    
        self.Nobs =  10                    
        self.ts = 0.2                            
        self.vmax = 1.5
        self.omega_max = 0.5
        self.q = 1
        self.qtheta = 1
        self.rv = 10
        self.rw = 10
        self.qN = 200
        self.qthetaN = 10
        self.throttle_ratio = 0.9
        self.build_directory = 'mpc_build'
        self.optimizer_name = 'navigation'
        self.bad_exit_codes = ["NotConvergedIterations", "NotConvergedOutOfTime"]