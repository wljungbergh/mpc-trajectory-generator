import yaml

""" 
    File that contains all the neccessary configuration parameters for the 
    MPC Trajectory Generation Module
"""
        
required_config_params = {
    'N_hor': 'The length of the receding horizon controller',
    'lin_vel_min': 'Vehicle contraint on the minimal velocity possible',
    'lin_vel_max': 'Vehicle contraint on the maximal velocity possible',
    'lin_acc_min': 'Vehicle contraint on the maximal linear retardation',
    'lin_acc_max': 'Vehicle contraint on the maximal linear acceleration',
    'ang_vel_max': 'Vehicle contraint on the maximal angular velocity',
    'ang_acc_max': 'Vehicle contraint on the maximal angular acceleration (considered to be symmetric)',
    'throttle_ratio': 'What percent of the maximal velocity should we try to',
    'num_steps_taken': 'How many steps should be taken from each mpc-solution. Range (1 - N_hor)',
    'ts': 'Size of the time-step',
    'vel_red_steps' :'Steps for velocity reduction',
    'lin_vel_penalty': 'Cost for linear velocity control action',
    'lin_acc_penalty': 'Cost for linear acceleration', 
    'ang_vel_penalty': 'Cost angular velocity control action',
    'ang_acc_penalty': 'Cost angular acceleration',
    'cte_penalty': 'Cost for cross-track-error from each line segment',
    'q': 'Cost for each position relative the final reference position',
    'qv': 'Cost for speed deviation at each time step',
    'qtheta': 'Cost for each heading relative to the final refernce position',
    'qN': 'Terminal cost; error relative to final reference position',
    'qthetaN': 'Terminal cost; error relative to final reference heading',
    'nx': 'Number of states for the robot (x,y,theta)',
    'nz': 'Number of optimization parameters',
    'nu': 'Number of control inputs',
    'nobs': 'Number of variables per obstacles',
    'Nobs': 'Maximal number of obstacles',
    'Ndynobs': 'Maximal number of dynamic obstacles',
    'vehicle_width': 'Vehicle width in meters',
    'vehicle_margin': 'Extra margin used for padding in meters',
    'build_type': "Can have 'debug' or 'release'",
    'build_directory': 'Name of the directory where the build is created',
    'bad_exit_codes': 'Optimizer specific names',
    'optimizer_name': 'Optimizer type, default "navigation"'
}


class dotdict(dict):
    """dot.notation access to dictionary attributes"""
    __getattr__ = dict.get
    __setattr__ = dict.__setitem__
    __delattr__ = dict.__delitem__

class Configurator:
    def __init__(self,yaml_fp):
        self.fp = yaml_fp
        print(f"[CONFIG] Loading configuration from '{self.fp}'")
        with open(self.fp, 'r') as stream:
            self.input = yaml.safe_load(stream)
        self.args = dotdict()

    def configurate_key(self, key):
        value = self.input.get(key)
        if value is None:
            print(f"[CONFIG] Can not find '{key}' in the YAML-file. Explanation is: '{required_config_params[key]}'")
            raise RuntimeError('[CONFIG] Configuration is not properly set')
        self.args[key] = value

    def configurate(self):
        print('[CONFIG] STARTING CONFIGURATION...')
        for key in required_config_params:
            self.configurate_key(key)
        print('[CONFIG] CONFIGURATION FINISHED SUCCESSFULLY...')
        return self.args
