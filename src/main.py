from path_generator import PathGenerator
from visibility.graphs import Graphs
import math
import matplotlib.pyplot as plt

graphs = Graphs()
g = graphs.get_graph(complexity=2)
 

path_gen = PathGenerator(build=False)
start = list(g.start) + [math.radians(45)]
end = list(g.end) + [math.radians(270)]
xx,xy,uv,uomega = path_gen.run(g, start, end)

path_gen.plot_result(xx,xy,uv,uomega)
plt.show()
