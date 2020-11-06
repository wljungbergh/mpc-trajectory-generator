import extremitypathfinder as epf
from extremitypathfinder.plotting import PlottingEnvironment, draw_prepared_map
from extremitypathfinder import PolygonEnvironment
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

def plot_boundaries(boundary_coordinates, c = 'r'):
    plt.subplot(313)
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
        
        # Pre-proccess obstacles
        obstacle_list = self.preprocess_obstacles(obstacle_list)

        # Give obstacles and boundaries to environment
        self.env.store(boundary_coordinates, obstacle_list)

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

    def preprocess_obstacles(self, obstacle_list):
        return obstacle_list
    
    def step(self, state, vel, omega, dt):
        return simulate_step(state, vel, omega, self.config.dt)

    def point2point_sovle(self, state, start_point, end_point):
        commands = []

        # Deviation variables
        dx = end_point[0] - start_point[0]
        dy = end_point[1] - start_point[1]
        # Distance of segment
        dist = math.sqrt(dx**2 + dy**2)
        # Angle of segment
        line_segment_angle = math.atan2(dy, dx)
        # Angle of 
        angle_diff = line_segment_angle - state[2]
        if angle_diff != 0:
            direction = abs(angle_diff) / angle_diff
            # Take N full omega time step with no vel
            n_time_steps = abs(angle_diff) / (self.config.omega_max * self.config.dt)
            commands += [0, direction * self.config.omega_max] * math.floor(n_time_steps)
            commands += [0, direction * self.config.omega_max * abs(n_time_steps) % 1]
        # Take N full vel time step with no omega
        n_time_steps = dist / (self.config.v_max * self.config.dt)
        commands += [self.config.v_max,0] * math.floor(n_time_steps)
        commands += [self.config.v_max * abs(n_time_steps) % 1,0]
        # Update state and and (0,0) for compatability
        state[2] = line_segment_angle
        state[0] = end_point[0]
        state[1] = end_point[0]
        commands += [0,0]

        return commands

    def generate_naive_warmstart(self, initial_pose, inital_path):
        # state = [x y heading]
        # THIS METHOD WILL BE USED TO GENERATE THE DECISION VARIABLES (vel and omega) for the initual guess generated by the A* sovler

        # Set the state of the robot to the initial pose
        state = initial_pose
        # Initialize the control commands list
        inital_control_commands = []

        for i in range(len(inital_path) - 1):
            sub_list_commands = self.point2point_sovle(state, inital_path[i], inital_path[i+1])
            inital_control_commands += sub_list_commands


        return inital_control_commands

    def get_action(self, state, next_waypoint):
        # Deviation variables
        dx = end_point[0] - state[0]
        dy = end_point[1] - state[1]
        # Distance of segment
        dist = math.sqrt(dx**2 + dy**2)
        # Angle of segment
        line_segment_angle = math.atan2(dy, dx)
        # Angle of 
        angle_diff = line_segment_angle - state[2]
        direction = abs(angle_diff) / angle_diff

        if dist >= v_max * dt:
            vel = v_max
        else:
            vel = dist / dt
        
        if abs(angle_diff) > omega_max * dt:
            direction = abs(angle_diff) / angle_diff
            omega = omega_max * direction
        else:
            omega = angle_diff / dt
            
    def generate_warmstart(self, inital_pose, initial_path):
        self.get_action(state)

if __name__ == '__main__':

    graphs = Graphs()

    g = graphs.get_graph(1)

    ppp = PathPreProcessor(g.boundary_coordinates, g.obstacle_list, plotting = False)

    path, _ = ppp.get_initial_guess(g.start, g.end)

    inital_pose = list(g.start) + [math.pi/4]
    commands = ppp.generate_naive_warmstart(inital_pose.copy(), path)

    plot_trajectory(inital_pose, commands, g.obstacle_list, g.end)
    plot_boundaries(g.boundary_coordinates,c = 'r')
    plot_commands(commands)
    plt.show()