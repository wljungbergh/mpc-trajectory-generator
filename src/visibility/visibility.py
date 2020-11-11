import extremitypathfinder.extremitypathfinder as epf
from extremitypathfinder.plotting import PlottingEnvironment, draw_prepared_map
from extremitypathfinder.extremitypathfinder import PolygonEnvironment
from graphs import Graphs
import pyclipper
import math
import numpy as np
import matplotlib.pyplot as plt
import timeit

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

def plot_verticies(verticies, radius,  ax):
    for vert in verticies:
        c = plt.Circle(vert, radius, color = 'r', alpha = 0.7)
        ax.add_artist(c)
    pass

class Config:
    def __init__(self):
        self.dt = 0.1
        self.omega_max = 1
        self.v_max = 1.5
        self.vehicle_width = 0.5


class PathPreProcessor:
    def __init__(self, boundary_coordinates, obstacle_list, padding_distance = 0.25, plotting = False, config = Config()):
        self.config = config
        self.inflator = pyclipper.PyclipperOffset()
                # Define environment based on wheter or not to plot
        if plotting:
            self.env = PlottingEnvironment(plotting_dir='plotted_paths')
        else: 
            self.env = PolygonEnvironment()

        # Obstacles
        self.original_obstacle_list = obstacle_list.copy()
        # Pre-proccess obstacles
        self.processed_obstacle_list = self.preprocess_obstacles(obstacle_list)
        # Boundary
        self.original_boundary_coordinates = boundary_coordinates.copy()
        # Pre-process boundaries
        self.processed_boundary_coordinates = self.preprocess_obstacle(
                                                pyclipper.scale_to_clipper(boundary_coordinates), 
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
            path, dist: path is a list of coordinates linearly connected which is the inital guess, and dist is the distance of the inital guess 
        """
        path, distance = self.env.find_shortest_path(start_pos, end_pos)
        return path, distance

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
    
    def find_original_verticies(self, path):
        verticies = []
        if not (len(path) > 2):
           print('[INFO] Path is only one line. No verticies to find.') 
           return verticies

        all_vert = self.original_obstacle_list + [self.original_boundary_coordinates]

        all_vert_flat = [item for sublist in all_vert for item in sublist]
        for vert in path[1:-1]: # dont use start and final positions as contstraints
            best_vert = ()
            best_dist = 10e3
            for ref_vert in all_vert_flat:
                dist = np.linalg.norm((np.asarray(vert) - np.asarray(ref_vert)), ord = 2)
                if dist < best_dist: 
                    best_dist = dist
                    best_vert = ref_vert
            
            verticies.append(best_vert)

        return verticies
    

if __name__ == '__main__':

    graphs = Graphs()

    g = graphs.get_graph(complexity=0)

    ppp = PathPreProcessor(g.boundary_coordinates, g.obstacle_list, plotting = False)
    path, _ = ppp.get_initial_guess(g.start, g.end)
    verticies = ppp.find_original_verticies(path)

    fig, ax = plt.subplots()
    plot_boundaries(ppp.original_boundary_coordinates, ax, c='k')
    plot_boundaries(ppp.processed_boundary_coordinates, ax, c='g')
    plot_obstacles(ppp.original_obstacle_list, c='r', ax = ax)
    plot_obstacles(ppp.processed_obstacle_list,c='b', ax = ax)
    plot_path(path, c='-ok', ax = ax)
    plot_verticies(verticies, radius = ppp.config.vehicle_width, ax = ax)
    plt.axis('equal')
    plt.show()
    
    