from visibility.visibility import PathPreProcessor
from mpc.mpc_generator import MpcModule
from utils import Config

class PathGenerator:
    """ Class responsible for generating a smooth trajectory based on inital preproccsing 
        together with mpc-solver. Uses a configuration specified in utils/config.py
    """
    def __init__(self, config):
        self.config = Config()
        self.ppp = PathPreProcessor(config)
        self.mpc_generator = MpcModule(config)

    
    def run(self, map, start, end):
        raise NotImplementedError('Here is where the magic will happen')