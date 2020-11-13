import time
from os import makedirs
from os.path import abspath, exists, join

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

from extremitypathfinder.extremitypathfinder import PolygonEnvironment

EXPORT_RESOLUTION = 200  # dpi
EXPORT_SIZE_X = 19.0  # inch
EXPORT_SIZE_Y = 11.0  # inch

POLYGON_SETTINGS = {
    'edgecolor': 'black',
    'fill': False,
    'linewidth': 1.0,
}

SHOW_PLOTS = False
# TODO avoid global variable
PLOTTING_DIR = 'all_plots'


def get_plot_name(file_name='plot'):
    return abspath(join(PLOTTING_DIR, file_name + '_' + str(time.time())[:-7] + '.png'))


def export_plot(fig, file_name):
    fig.set_size_inches(EXPORT_SIZE_X, EXPORT_SIZE_Y, forward=True)
    plt.savefig(get_plot_name(file_name), dpi=EXPORT_RESOLUTION)
    plt.close()


def mark_points(vertex_iter, **kwargs):
    try:
        coordinates = [v.coordinates.tolist() for v in vertex_iter]
    except AttributeError:
        coordinates = [v for v in vertex_iter]
    coords_zipped = list(zip(*coordinates))
    if coords_zipped:  # there might be no vertices at all
        plt.scatter(*coords_zipped, **kwargs)


def draw_edge(v1, v2, c, alpha, **kwargs):
    if type(v1) == tuple:
        x1, y1 = v1
        x2, y2 = v2
    else:
        x1, y1 = v1.coordinates
        x2, y2 = v2.coordinates
    plt.plot([x1, x2], [y1, y2], color=c, alpha=alpha, **kwargs)


def draw_polygon(ax, coords, **kwargs):
    kwargs.update(POLYGON_SETTINGS)
    polygon = Polygon(coords, **kwargs)
    ax.add_patch(polygon)


def draw_boundaries(map, ax):
    # TODO outside light grey
    # TODO fill holes light grey
    draw_polygon(ax, map.boundary_polygon.coordinates)
    for h in map.holes:
        draw_polygon(ax, h.coordinates, facecolor='grey', fill=True)

    mark_points(map.all_vertices, c='black', s=15)
    mark_points(map.all_extremities, c='red', s=50)


def draw_internal_graph(map, ax):
    for start, all_goals in map.graph.get_neighbours():
        for goal in all_goals:
            draw_edge(start, goal, c='red', alpha=0.2, linewidth=2)


def set_limits(map, ax):
    ax.set_xlim((min(map.boundary_polygon.coordinates[:, 0]) - 1, max(map.boundary_polygon.coordinates[:, 0]) + 1))
    ax.set_ylim((min(map.boundary_polygon.coordinates[:, 1]) - 1, max(map.boundary_polygon.coordinates[:, 1]) + 1))


def draw_path(vertex_path):
    # start, path and goal in green
    if vertex_path:
        mark_points(vertex_path, c='g', alpha=0.9, s=50)
        mark_points([vertex_path[0], vertex_path[-1]], c='g', s=100)
        v1 = vertex_path[0]
        for v2 in vertex_path[1:]:
            draw_edge(v1, v2, c='g', alpha=1.0)
            v1 = v2


def draw_loaded_map(map):
    fig, ax = plt.subplots()

    draw_boundaries(map, ax)
    set_limits(map, ax)
    export_plot(fig, 'map_plot')
    if SHOW_PLOTS:
        plt.show()


def draw_prepared_map(map):
    fig, ax = plt.subplots()

    draw_boundaries(map, ax)
    draw_internal_graph(map, ax)
    set_limits(map, ax)
    export_plot(fig, 'prepared_map_plot')
    if SHOW_PLOTS:
        plt.show()


def draw_with_path(map, temp_graph, vertex_path):
    fig, ax = plt.subplots()

    start, goal = vertex_path[0], vertex_path[-1]
    draw_boundaries(map, ax)
    draw_internal_graph(map, ax)
    set_limits(map, ax)

    # additionally draw:
    # new edges yellow
    if start in temp_graph.get_all_nodes():
        for n2, d in temp_graph.neighbours_of(start):
            draw_edge(start, n2, c='y', alpha=0.7)

    all_nodes = temp_graph.get_all_nodes()
    if goal in all_nodes:
        # edges only run towards goal
        for n1 in all_nodes:
            if goal in temp_graph.get_neighbours_of(n1):
                draw_edge(n1, goal, c='y', alpha=0.7)

    # start, path and goal in green
    draw_path(vertex_path)

    export_plot(fig, 'graph_path_plot')
    if SHOW_PLOTS:
        plt.show()


def draw_only_path(map, vertex_path):
    fig, ax = plt.subplots()

    draw_boundaries(map, ax)
    set_limits(map, ax)
    draw_path(vertex_path)

    export_plot(fig, 'path_plot')
    if SHOW_PLOTS:
        plt.show()


def draw_graph(map, graph):
    fig, ax = plt.subplots()

    all_nodes = graph.get_all_nodes()
    mark_points(all_nodes, c='black', s=30)

    for n in all_nodes:
        x, y = n.coordinates
        neighbours = graph.get_neighbours_of(n)
        for n2 in neighbours:
            x2, y2 = n2.coordinates
            dx, dy = x2 - x, y2 - y
            plt.arrow(x, y, dx, dy, head_width=0.15, head_length=0.5, head_starts_at_zero=False, shape='full',
                      length_includes_head=True)

    set_limits(map, ax)

    export_plot(fig, 'graph_plot')
    if SHOW_PLOTS:
        plt.show()


class PlottingEnvironment(PolygonEnvironment):

    def __init__(self, plotting_dir=PLOTTING_DIR):
        super().__init__()
        global PLOTTING_DIR
        PLOTTING_DIR = plotting_dir
        if not exists(plotting_dir):
            makedirs(plotting_dir)

    def store(self, *args, **kwargs):
        super().store(*args, **kwargs)
        draw_loaded_map(self)

    def prepare(self):
        super().prepare()
        draw_prepared_map(self)

    def find_shortest_path(self, *args, **kwargs):
        # important to not delete the temp graph! for plotting
        vertex_path, distance = super().find_shortest_path(*args, free_space_after=False, **kwargs)

        if self.temp_graph:  # in some cases (e.g. direct path possible) no graph is being created!
            draw_graph(self, self.temp_graph)
            draw_with_path(self, self.temp_graph, vertex_path)
            draw_only_path(self, vertex_path)
            del self.temp_graph  # free the memory

        # extract the coordinates from the path
        return vertex_path, distance
