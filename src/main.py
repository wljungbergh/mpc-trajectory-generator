from path_generator import PathGenerator
from visibility.graphs import Graphs
import math
import matplotlib.pyplot as plt
from utils.config import Configurator
from pathlib import Path
import os

graphs = Graphs()
g = graphs.get_graph(complexity=2)
file_path = Path(__file__)

config_fn = 'default.yaml'
yaml_fp = os.path.join(str(file_path.parent.parent), 'configs', config_fn)
configurator = Configurator(yaml_fp)
config = configurator.configurate()

path_gen = PathGenerator(config, build=False)

start = list(g.start)
end = list(g.end)
xx,xy,uv,uomega,tot_solver_time,overhead_times = path_gen.run(g, start, end)

path_gen.plot_results(xx,xy,uv,uomega, start, end, dynamic=False)
plt.show()
