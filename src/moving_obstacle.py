from path_generator import PathGenerator
from visibility.graphs import Graphs
import math
from matplotlib.gridspec import GridSpec
from matplotlib.lines import Line2D
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
from utils.config import Configurator
from pathlib import Path
import os
import numpy as np



def generate_obstacle(p1, p2, freq, time):
    p1 = np.array(p1)
    p2 = np.array(p2)
    time = np.array(time)
    t = 0.5*np.sin(freq * time)+0.5
    if type(t) == np.ndarray:
        t = np.expand_dims(t,1)
   
    p3 = t*p1 + (1-t)*p2
    return p3

graphs = Graphs()
g = graphs.get_graph(complexity=1)
file_path = Path(__file__)

config_fn = 'jconf_1.yaml'
yaml_fp = os.path.join(str(file_path.parent.parent), 'configs', config_fn)
configurator = Configurator(yaml_fp)
config = configurator.configurate()

path_gen = PathGenerator(config, build=False)

start = list(g.start) + [math.radians(0)]
end = list(g.end) + [math.radians(0)]

obstacle_radius = 0.5
#obs_pos = [(x, 5.0, obstacle_radius+config.vehicle_width/2+config.vehicle_margin) for x in np.linspace(10,5,int(150/config.ts))]
p1 = [6, 5.5]
p2 = [4.5, 7]
freq = 0.1
obs_pos = [(generate_obstacle(p1[0], p2[0], freq, t), generate_obstacle(p1[1], p2[1], freq, t), obstacle_radius+config.vehicle_width/2+config.vehicle_margin) for t in np.linspace(0,150,1000)]
#obs_pos1 = [(8.0, y, obstacle_radius+config.vehicle_width/2+config.vehicle_margin) for y in np.linspace(5.5,2,int(150/config.ts))]
p1 = [17.5, 43]
p2 = [22, 37.5]
freq = 0.1
obs_pos1 = [(generate_obstacle(p1[0], p2[0], freq, t), generate_obstacle(p1[1], p2[1], freq, t), obstacle_radius+config.vehicle_width/2+config.vehicle_margin) for t in np.linspace(0,150,1000)]

p1 = [40.5, 18]
p2 = [37, 26]
freq = 0.1
obs_pos2 = [(generate_obstacle(p1[0], p2[0], freq, t), generate_obstacle(p1[1], p2[1], freq, t), obstacle_radius+config.vehicle_width/2+config.vehicle_margin) for t in np.linspace(0,150,1000)]
dyn_obs_list = [obs_pos, obs_pos1, obs_pos2]

xx,xy,uv,uomega,tot_solver_time = path_gen.run(g, start, end)



fig = plt.figure(constrained_layout = True)
gs = GridSpec(1, 1, figure=fig)

path_ax =  fig.add_subplot(gs[0,0])



path_ax.plot(start[0], start[1], marker='*', color='g', markersize = 15, label='Start')
path_ax.plot(end[0], end[1], marker='*', color='r', markersize = 15,label='End')
path_gen.ppp.plot_all(path_ax)
path_line, = path_ax.plot([1], '-ob', alpha =0.7, markersize=5)
path_ax.set_xlabel('X [m]', fontsize = 15)
path_ax.set_ylabel('Y [m]', fontsize = 15)
path_ax.axis('equal')

legend_elems = [  Line2D([0], [0], color='k', label='Original Boundary' ),
                    Line2D([0], [0], color='g', label='Padded Boundary'),
                    Line2D([0], [0], marker='o', color='b', label='Traversed Path', alpha = 0.5),
                    Line2D([0], [0], marker='*', color='g', label='Start Position', alpha = 0.5),
                    Line2D([0], [0], marker='*', color='r', label='End Position'),
                    mpatches.Patch(color='b', label='Robot'),
                    mpatches.Patch(color='r', label='Obstacle'),
                    mpatches.Patch(color='y', label='Padded obstacle')
                            ]
path_ax.legend(handles = legend_elems)
obs = [object] * len(g.dyn_obs_list)
obs_padded = [object] * len(g.dyn_obs_list)

for i in range(len(xx)):
    path_line.set_data(xx[:i], xy[:i])
    veh = plt.Circle((xx[i], xy[i]), config.vehicle_width/2, color = 'b', alpha = 0.7, label='Robot')
    path_ax.add_artist(veh)
    for j, obstacle in enumerate(g.dyn_obs_list):
        p1,p2,freq,obstacle_radius = obstacle
        pos = path_gen.ppp.generate_obstacle(p1,p2, freq, i * config.ts)
        obs[j] = plt.Circle(pos, obstacle_radius, color = 'r', alpha = 1, label='Obstacle')
        obs_padded[j] = plt.Circle(pos, obstacle_radius + config.vehicle_width/2 + config.vehicle_margin, color = 'y', alpha = 0.7, label='Padded obstacle')
        path_ax.add_artist(obs_padded[j])
        path_ax.add_artist(obs[j])
    

    
    
    plt.draw()
    plt.pause(self.config.ts / 10)
    veh.remove()    
    for j in range(len(g.dyn_obs_list)):
        obs[j].remove()
        obs_padded[j].remove()
    

plt.show()