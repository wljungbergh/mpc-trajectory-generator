import extremitypathfinder.extremitypathfinder as epf
from extremitypathfinder.plotting import PlottingEnvironment, draw_prepared_map
from extremitypathfinder.extremitypathfinder import PolygonEnvironment
from graphs import Graphs
import pyclipper
import math
import numpy as np
import matplotlib.pyplot as plt

def simulate_step(state, vel, omega, dt):
        state[0] += dt * math.cos(state[2]) * vel
        state[1] += dt * math.sin(state[2]) * vel
        state[2] += dt * omega
        return state

def plot_obstacles(obstacle_list, c):
    for obstacle in obstacle_list:
        for i in range(len(obstacle) - 1):
            point1 = obstacle[i]
            point2 = obstacle[i + 1]
            plt.plot([point1[0], point2[0]], [point1[1], point2[1]], c)
        point1 = obstacle[-1]
        point2 = obstacle[0]
        plt.plot([point1[0], point2[0]], [point1[1], point2[1]], c)

def plot_trajectory(inital_pose, commands, obstacle_list, end,  padded_obstacles = None):
        N = len(commands) // 2
        state = inital_pose
        startx = inital_pose[0]
        starty = inital_pose[1]
        states = []
        states.append(inital_pose.copy())

        for t in range(0, N):
            vel = commands[2*t]
            omega = commands[2*t + 1]
            state = simulate_step(state, vel, omega, 0.1)
            states.append(state.copy())



        x = [t[0] for t in states]
        y = [t[1] for t in states]  
        plt.subplot(313)
        plt.plot(startx, starty, 'ro')
        plt.plot(x, y, '-')
        plt.plot(end[0], end[1], 'ro')
        plot_obstacles(obstacle_list, c = 'k')
        if not padded_obstacles is None:
            plot_obstacles(obstacle_list, c = 'g')

        #plt.axis('equal')

def plot_boundaries(boundary_coordinates, c = 'g'):
    plot_obstacles([boundary_coordinates], c = c)

def plot_commands(commands, dt = 0.1):
    N = len(commands) // 2
    time = np.arange(0, dt*N, dt)
    vel = commands[::2]
    omega = commands[1::2]

    plt.subplot(311)
    plt.plot(time, vel, '-o')
    plt.ylabel('velocity')
    plt.subplot(312)
    plt.plot(time, omega, '-o')
    plt.ylabel('angular velocity')
    plt.xlabel('Time')

def plot_path(path, c):
    for i in range(len(path)-1):
        plt.plot([path[i][0], path[i+1][0]],[path[i][1], path[i+1][1]],c)


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
    
    

if __name__ == '__main__':

    graphs = Graphs()

    g = graphs.get_graph(complexity=0)

    ppp = PathPreProcessor(g.boundary_coordinates, g.obstacle_list, plotting = False)
    path, _ = ppp.get_initial_guess(g.start, g.end)

    plot_boundaries(ppp.original_boundary_coordinates,c='k')
    plot_boundaries(ppp.processed_boundary_coordinates,c='g')
    plot_obstacles(ppp.original_obstacle_list, c='r')
    plot_obstacles(ppp.processed_obstacle_list,c='b')
    plot_path(path, c='-ok')

    plt.show()
    
    