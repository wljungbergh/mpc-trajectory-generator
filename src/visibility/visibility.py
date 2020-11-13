import extremitypathfinder.extremitypathfinder as epf
from extremitypathfinder.plotting import PlottingEnvironment, draw_prepared_map
from extremitypathfinder.extremitypathfinder import PolygonEnvironment
from graphs import Graphs
import pyclipper
import math
import numpy as np
import matplotlib.pyplot as plt
import timeit

# Helper functions 
def plot_obstacles(obstacle_list, c, ax):
    for obstacle in obstacle_list:
        for i in range(len(obstacle) - 1):
            point1 = obstacle[i]
            point2 = obstacle[i + 1]
            plt.plot([point1[0], point2[0]], [point1[1], point2[1]], c)
        point1 = obstacle[-1]
        point2 = obstacle[0]
        ax.plot([point1[0], point2[0]], [point1[1], point2[1]], c)

def plot_boundaries(boundary_coordinates, ax, c = 'g'):
    plot_obstacles([boundary_coordinates], c = c, ax = ax)

def plot_path(path, c, ax):
    for i in range(len(path)-1):
        ax.plot([path[i][0], path[i+1][0]],[path[i][1], path[i+1][1]],c)

def plot_vertices(vertices, radius,  ax):
    for vert in vertices:
        c = plt.Circle(vert, radius, color = 'r', alpha = 0.7)
        ax.add_artist(c)
    pass



#########
class Config:
    def __init__(self):
        self.vehicle_width = 0.5


class PathPreProcessor:
    def __init__(self, config, plotting = False):
        self.config = config
        self.inflator = pyclipper.PyclipperOffset()
                # Define environment based on wheter or not to plot
        if plotting:
            self.env = PlottingEnvironment(plotting_dir='plotted_paths')
        else: 
            self.env = PolygonEnvironment()

    def prepare(self, graph_map):

        # Obstacles
        self.original_obstacle_list = graph_map.obstacle_list.copy()
        # Pre-proccess obstacles
        self.processed_obstacle_list = self.preprocess_obstacles(graph_map.obstacle_list)
        # Boundary
        self.original_boundary_coordinates = graph_map.boundary_coordinates.copy()
        # Pre-process boundaries
        self.processed_boundary_coordinates = self.preprocess_obstacle(
                                                pyclipper.scale_to_clipper(graph_map.boundary_coordinates), 
                                                pyclipper.scale_to_clipper(-self.config.vehicle_width))

        # Give obstacles and boundaries to environment
        self.env.store(self.processed_boundary_coordinates, self.processed_obstacle_list)

        # Prepare the visibility graph 
        self.env.prepare()

    def get_initial_guess(self, start_pos, end_pos):
        """Method that generates the initially guessed path based on obstacles and boundaries specified during creation

        Args:
            start_pos (tuple): tuple containing x and y position for starting position
            end_pos (tuple): tuple containing x and y position for end position

        Returns:
            path ([tuples]) : a list of coordinates that together becomes the inital path. These points lies on extremities of the padded obstacles 
            vertices ([tuples]) : a list of the vertices on the original (unpadded) obstacles corresponding to the vertices in path
            
            """
        path, distance = self.env.find_shortest_path(start_pos, end_pos)
        vertices = self.find_original_vertices(path)

        self.path = path
        self.vert = vertices
        self.vert_copy = vertices.copy()

        return path, vertices

    def preprocess_obstacle(self, obstacle, inflation, boundary = False):
        self.inflator.Clear()
        self.inflator.AddPath(obstacle, pyclipper.JT_MITER, pyclipper.ET_CLOSEDPOLYGON)
        inflated_obstacle = pyclipper.scale_from_clipper(self.inflator.Execute(inflation))[0]
        return inflated_obstacle    
    
    def preprocess_obstacles(self, obstacle_list):
        inflation = pyclipper.scale_to_clipper(self.config.vehicle_width)
        inflated_obstacles = []
        for obs in obstacle_list:
            obstacle = pyclipper.scale_to_clipper(obs)
            inflated_obstacle = self.preprocess_obstacle(obstacle, inflation, boundary=False)
            inflated_obstacle.reverse() # obstacles are ordered clockwise
            inflated_obstacles.append(inflated_obstacle)

        return inflated_obstacles
    
    @staticmethod
    def dist_between_points(p1, p2):
        return np.linalg.norm((np.asarray(p1) - np.asarray(p2)), ord = 2)
    
    def get_closest_vert(self, vert, list_to_compare):
        best_vert = ()
        best_dist = 10e3
        best_idx = 0
        for idx, ref_vert in enumerate(list_to_compare):
            dist = self.dist_between_points(vert, ref_vert)
            if dist < best_dist: 
                best_dist = dist
                best_idx = idx
                best_vert = ref_vert

        return best_vert, best_idx
        
    def find_original_vertices(self, path):
        vertices = []
        if not (len(path) > 2):
           print('[INFO] Path is only one line. No vertices to find.') 
           return vertices

        all_vert = self.original_obstacle_list + [self.original_boundary_coordinates]

        all_vert_flat = [item for sublist in all_vert for item in sublist]
        for vert in path[1:-1]: # dont use start and final positions as contstraints
            closest_vert, _ = self.get_closest_vert(vert, all_vert_flat)
            vertices.append(closest_vert)

        return vertices
    
    def find_closest_vertices(self, current_pos, n_vertices = 10, N_STEPS_LOOK_BACK = 2):
        _, idx = self.get_closest_vert(current_pos, self.vert)
        lb = max(0, idx - N_STEPS_LOOK_BACK) # look two objects behind 
        ub = min(len(self.vert), n_vertices - N_STEPS_LOOK_BACK)
        return self.vert[lb:ub].copy()

        ###### If one wants to use euclidian distance instead.
        # self.vert_copy.sort(key = lambda x: self.dist_between_points(current_pos, x))
        # return self.vert.copy()[:n_vertices]
        ######

    def plot_all(self, ax):
        plot_boundaries(self.original_boundary_coordinates, ax, c='k')
        plot_boundaries(self.processed_boundary_coordinates, ax, c='g')
        plot_obstacles(ppp.original_obstacle_list, c='r', ax = ax)
        plot_obstacles(self.processed_obstacle_list,c='b', ax = ax)
        plot_path(self.path, c='-ok', ax = ax)
        plot_vertices(self.vert, radius = ppp.config.vehicle_width, ax = ax)
    

if __name__ == '__main__':

    graphs = Graphs()

    g = graphs.get_graph(complexity=0)
    config = Config()
    ppp = PathPreProcessor(config, plotting = False)
    ppp.prepare(g)
    path, vertices = ppp.get_initial_guess(g.start, g.end)
    vert = ppp.find_closest_vertices(g.end)

    fig, ax = plt.subplots()
    
    
    
    plt.axis('equal')
    plt.show()
    
    