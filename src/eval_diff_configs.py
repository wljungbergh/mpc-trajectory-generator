from path_generator import PathGenerator
from visibility.graphs import Graphs
import math
import matplotlib.pyplot as plt
from utils.config import Configurator
from pathlib import Path
import os

graphs = Graphs()
g = graphs.get_graph(complexity=1)
file_path = Path(__file__)

start = list(g.start) + [math.radians(45)]
end = list(g.end) + [math.radians(0)]


config_filenames = ['william_config.yaml']
for config_filename in config_filenames:
    yaml_fp = os.path.join(str(file_path.parent.parent), 'configs', config_filename)
    
    configurator = Configurator(yaml_fp)
    config = configurator.configurate()

    path_gen = PathGenerator(config, build=False)

    xx,xy,uv,uomega = path_gen.run(g, start, end)
    path_gen.plot_result(xx,xy,uv,uomega, start, end)

    plt.show()
    plt.close('all')
