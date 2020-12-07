import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection

class Graph:
    def __init__(self, boundary_coordinates, obstacle_list, default_start, default_end):
        self.boundary_coordinates = boundary_coordinates
        self.obstacle_list = obstacle_list
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

        self.graphs.append(Graph(boundary_coordinates, obstacle_list, (1,1), (8,8)))
        
        ############### Second Graph ############################# 
        # To be specified in counter-clockwise ordering
        boundary_coordinates = [(0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0)]

        # To be specified in clock-wise ordering
        obstacle_list = [
            [(5.0, 0.0), (5.0, 15.0), (7.0, 15.0), (7.0, 0.0)],
            [(12.0, 12.5),(12.0,20.0),(15.0,20.0),(15.0,12.5)], 
            [(12.0, 0.0),(12.0,7.5),(15.0,7.5),(15.0, 0.0)],
         ]

        self.graphs.append(Graph(boundary_coordinates, obstacle_list, (1,5), (19,10)))

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

        self.graphs.append(Graph(boundary_coordinates, obstacle_list, (1,1), (5,20)))

        ############### Third Graph ############################# 
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

        self.graphs.append(Graph(boundary_coordinates, obstacle_list, (30,5), (30,55)))


        self.min_complexity = 0
        self.max_complexity = len(self.graphs) - 1




    def get_graph(self, complexity):
        if not (self.min_complexity <= complexity <= self.max_complexity):
            raise ValueError(f'Complexity should be in the range {self.min_complexity } - {self.max_complexity}')

        return self.graphs[complexity]


