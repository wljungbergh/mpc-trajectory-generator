import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
import math

class Graph:
    def __init__(self, boundary_coordinates, obstacle_list, default_start, default_end, dyn_obs_list = []):
        self.boundary_coordinates = boundary_coordinates
        self.obstacle_list = obstacle_list
        self.dyn_obs_list = dyn_obs_list
        self.start = default_start
        self.end = default_end


class Graphs: 
    def __init__(self):
        self.graphs = []

        ############### First Graph ############################# 
        # To be specified in counter-clockwise ordering
        boundary_coordinates = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]

        # To be specified in clock-wise ordering
        obstacle_list = [
            [(3.0, 3.0), (3.0, 7.0), (7.0, 7.0), (7.0, 3.0), ],
         ]

        self.graphs.append(Graph(boundary_coordinates, obstacle_list, (1,1, math.radians(0)), (8,8, math.radians(90))))
        
        ############### Second Graph ############################# 
        # To be specified in counter-clockwise ordering
        boundary_coordinates = [(0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0)]

        # To be specified in clock-wise ordering
        obstacle_list = [
            [(5.0, 0.0), (5.0, 15.0), (7.0, 15.0), (7.0, 0.0)],
            [(12.0, 12.5),(12.0,20.0),(15.0,20.0),(15.0,12.5)], 
            [(12.0, 0.0),(12.0,7.5),(15.0,7.5),(15.0, 0.0)],
         ]

        self.graphs.append(Graph(boundary_coordinates, obstacle_list, (1,5, math.radians(45)), (19,10, math.radians(0))))

        ############### Third Graph ############################# 
        # To be specified in counter-clockwise ordering
        boundary_coordinates = [(0.0, 0.0),(10.0,0.0), (10.0 ,10.0), (25.0, 10.0), (25.0,0.0), (50.0,0), (50,50), (0, 50), \
                                (0,16),(10,16),(10,45),(15,45),(15,30),(35,30),(35,15),(0,15)]


        # To be specified in clock-wise ordering
        obstacle_list = [
            [(30.0, 5.0), (30.0, 14.5), (40.0, 14.5), (40, 5.0)],
            [(45.0, 15.0),(44.0,20.0),(46.0,20.0)],
            [(25, 35),(25,40),(40, 40),(40,35)],
            [(32.0, 6.0), (32.0, 10.5), (42.0, 12.5), (42, 8.0)]
         ]

        dyn_obs_list = [
            [[6.5, 5], [4.5, 7], 0.1, 0.5], 
            [[17.5, 43], [22, 37.5], 0.1, 0.5], 
            [[40.5, 18], [37, 26], 0.1, 0.5]
        ]

        self.graphs.append(Graph(boundary_coordinates, obstacle_list, (1,1,math.radians(45)), (5,20,math.radians(270)), dyn_obs_list))

        ############### Forth Graph ############################# 
        # To be specified in counter-clockwise ordering
        boundary_coordinates = [(3.6, 57.8), (3.6, 3.0), (58.3, 3.0), (58.1, 58.3)]


        # To be specified in clock-wise ordering
        obstacle_list = [
                        [(21.1, 53.1), (21.4, 15.1), (9.3, 15.1), (9.1, 53.1)],
                        [(35.7, 52.2), (48.2, 52.3), (48.7, 13.6), (36.1, 13.8)], 
                        [(17.0, 50.5),(30.7, 50.3), (30.6, 45.0), (17.5, 45.1)],
                        [(26.4, 39.4), (40.4, 39.3), (40.5, 35.8), (26.3, 36.0)],
                        [(19.3, 31.7), (30.3, 31.6), (30.1, 27.7), (18.9, 27.7)],
                        [(26.9, 22.7), (41.4, 22.6), (41.1, 17.5), (27.4, 17.6)]
                        ]

        self.graphs.append(Graph(boundary_coordinates, obstacle_list, (30,5, math.radians(90)), (30,55, math.radians(90))))

        ############### Fifth Graph ############################# 
        # To be specified in counter-clockwise ordering
        boundary_coordinates = [(54.0, 57.8), (7.8, 57.5), (7.5, 17.9), (53.0, 17.0)]


        # To be specified in clock-wise ordering
        obstacle_list= [[(14.0, 57.6), (42.1, 57.6), (42.2, 52.0), (13.4, 52.0)], [(7.7, 49.1), (32.2, 49.0), (32.1, 45.3), (7.7, 45.8)], [(34.2, 53.0), (41.2, 53.1), (40.9, 31.7), (34.4, 31.9)], [(35.7, 41.7), (35.7, 36.8), (11.7, 39.8), (12.1, 44.0), (31.3, 43.3)], [(5.8, 37.6), (24.1, 35.0), (23.6, 29.8), (5.0, 31.8)], [(27.1, 39.7), (32.7, 39.0), (32.8, 24.7), (16.2, 20.9), (14.5, 25.9), (25.3, 26.7), (27.9, 31.4), (26.1, 39.2)]]
        self.graphs.append(Graph(boundary_coordinates, obstacle_list, (10.3, 55.8, math.radians(270)), (38.1, 25.0, math.radians(300))))
        

        ############### Fifth Graph ############################# 
        # To be specified in counter-clockwise ordering
        boundary_coordinates = [(0,0), (0,10.0), (16,10.0), (16, 0)]

        # To be specified in clock-wise ordering
        obstacle_list = []
        start_pos = (1,5, math.radians(0))
        end_pos = (15,5, math.radians(0))
        self.graphs.append(Graph(boundary_coordinates, obstacle_list, start_pos, end_pos))


        ############### Sixth Graph ############################# 
        # To be specified in counter-clockwise ordering
        boundary_coordinates = [(0.37, 0.32), (5.79, 0.31), (5.79, 5.18), (0.14, 5.26)]

        # To be specified in clock-wise ordering
        obstacle_list = [[(2.04, 0.28), (2.0, 3.8), (2.8, 3.81), (2.78, 0.29)]]
        start_pos = (1.01, 0.98, math.radians(90))
        end_pos = (3.82, 1.05, math.radians(270))
        self.graphs.append(Graph(boundary_coordinates, obstacle_list, start_pos, end_pos))


        ############### Seventh Graph ############################# 
        # NOTE: Not always working
        # To be specified in counter-clockwise ordering
        boundary_coordinates = [(1.55, 1.15), (29.0, 1.1), (29.0, 28.75), (0.85, 28.9), (0.85, 1.15)]

        # To be specified in clock-wise ordering
        obstacle_list = [[(5.6, 3.3), (5.75, 20.15), (18.35, 20.05), (18.35, 19.7), (7.25, 19.7), (7.05, 3.2)], [(13.85, 23.4), (21.25, 23.35), (21.1, 16.4), (6.9, 16.35), (6.7, 12.9), (23.45, 13.25), (23.4, 25.05), (13.0, 25.35)]]
        start_pos = (2.95, 13.5, math.radians(90))
        end_pos = (9.6, 18.1, math.radians(180))
        self.graphs.append(Graph(boundary_coordinates, obstacle_list, start_pos, end_pos))


        ############### Eigth Graph ############################# 
        # To be specified in counter-clockwise ordering
        boundary_coordinates = [(2.0, 1.08), (22.8, 1.12), (22.84, 19.16), (1.8, 19.24)]

        # To be specified in clock-wise ordering
        obstacle_list = [[(9.64, 5.28), (9.56, 10.72), (8.68, 11.88), (9.48, 12.2), (10.52, 10.96), (11.6, 12.12), (12.6, 11.36), (11.28, 10.4), (11.6, 0.56), (9.68, 0.68)]]
        start_pos = (7.16, 8.16, math.radians(90))
        end_pos = (12.72, 9.32, math.radians(265))
        self.graphs.append(Graph(boundary_coordinates, obstacle_list, start_pos, end_pos))

        ############### Ninth Graph ############################# 
        # To be specified in counter-clockwise ordering
        #NOTE: Does not always work. Gets stuck
        boundary_coordinates = [(0.96, 1.88), (22.88, 1.72), (22.92, 20.8), (0.64, 20.92)]

        # To be specified in clock-wise ordering
        obstacle_list = [[(9.12, 1.48), (8.8, 9.56), (9.76, 12.72), (10.8, 9.56), (11.08, 1.48)]]
        start_pos = (7.44, 6.16, math.radians(90))
        end_pos = (12.44, 6.4, math.radians(265))
        self.graphs.append(Graph(boundary_coordinates, obstacle_list, start_pos, end_pos))

        ############### Tenth Graph ############################# 
        # To be specified in counter-clockwise ordering

        boundary_coordinates = [(2.36, 1.6), (22.6, 1.84), (22.16, 21.04), (1.52, 20.88)]

        # To be specified in clock-wise ordering
        obstacle_list = [[(9.92, 1.24), (9.64, 8.52), (12.6, 10.44), (15.6, 8.76), (15.76, 1.08)]]
        start_pos = (7.08, 5.88, math.radians(90))
        end_pos = (17.8, 6.56, math.radians(265))
        self.graphs.append(Graph(boundary_coordinates, obstacle_list, start_pos, end_pos))

        

        self.min_complexity = 0
        self.max_complexity = len(self.graphs) - 1




    def get_graph(self, complexity):
        if not (self.min_complexity <= complexity <= self.max_complexity):
            raise ValueError(f'Complexity should be in the range {self.min_complexity } - {self.max_complexity}')

        return self.graphs[complexity]


