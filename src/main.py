from path_generator import PathGenerator
from visibility.graphs import Graphs

import math

graphs = Graphs()
g = graphs.get_graph(complexity=1)

path_gen = PathGenerator(build=False)
start = list(g.start) + [math.radians(90)]
end = list(g.end) + [math.radians(0)]
path_gen.run(g, start, end)