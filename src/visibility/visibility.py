import extremitypathfinder as epf
from extremitypathfinder.plotting import PlottingEnvironment
from extremitypathfinder import PolygonEnvironment
from graphs import Graphs
import pyclipper



class PathPreProcessor:
    def __init__(self, boundary_coordinates, obstacle_list, padding_distance = 0.25, plotting = False):

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
        path, distance = env.find_shortest_path(start_pos, end_pos)
        return path, distance

    def preprocess_obstacles(self, obstacle_list):
        self.inflator.AddPaths(pyclipper.scale_to)


if __name__ == '__main__':

    graphs = Graphs()

    g = graphs.get_grapth(2)

    ppp = PathPreProcessor(g.boundary_coordinates, g.obstacle_list, vehicle_width = 0.5, plotting = True)

    path, _ = ppp.get_initial_guess()

    print(path)