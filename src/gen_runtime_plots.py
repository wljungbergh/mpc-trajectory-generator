from path_generator import PathGenerator
from visibility.graphs import Graphs
import math
import numpy as np
import matplotlib.pyplot as plt
from utils.config import Configurator
from pathlib import Path
import os

graphs = Graphs()
file_path = Path(__file__)

config_fn = 'default.yaml'
yaml_fp = os.path.join(str(file_path.parent.parent), 'configs', config_fn)
configurator = Configurator(yaml_fp)
config = configurator.configurate()


runtime = []
overhead= []
for i in range(1,11):
    print(f'STARTING NUMBER {i}')
    path_gen = PathGenerator(config, build=False)
    g = graphs.get_graph(complexity=i)
    start = list(g.start)
    end = list(g.end)
    xx,xy,uv,uomega, solver_times, overhead_times = path_gen.run(g, start, end)
    loop_times = [x+y for x,y in zip(solver_times, overhead_times)]
    overhead += overhead_times
    runtime += solver_times

loop = [x+y for x,y in zip(overhead,runtime)]
path_gen.plot_solver_hist(overhead)
path_gen.plot_solver_hist(runtime)
path_gen.plot_solver_hist(loop)
plt.show()