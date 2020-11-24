from path_generator import PathGenerator
from visibility.graphs import Graphs
import math
import matplotlib.pyplot as plt
from utils.config import Configurator

graphs = Graphs()
g = graphs.get_graph(complexity=2)

yaml_fp = '/Users/wlj/Documents/Education/Chalmers/Chalmers5/LP2/ssy226/mpc-trajectory-generator/configs/default.yaml'
configurator = Configurator(yaml_fp)
config = configurator.configurate()

path_gen = PathGenerator(config, build=True)

start = list(g.start) + [math.radians(45)]
end = list(g.end) + [math.radians(270)]
xx,xy,uv,uomega = path_gen.run(g, start, end)

path_gen.plot_result(xx,xy,uv,uomega)
plt.show()
