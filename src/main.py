from path_generator import PathGenerator
from visibility.graphs import Graphs
import math

graphs = Graphs()
g = graphs.get_graph(complexity=2)
 
path_gen = PathGenerator(build=True)
start = list(g.start) + [math.radians(0)]
end = list(g.end) + [math.radians(0)]
path_gen.run(g, start, end)