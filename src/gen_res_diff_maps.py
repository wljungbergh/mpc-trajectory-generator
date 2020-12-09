from path_generator import PathGenerator
from visibility.graphs import Graphs
import math
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from utils.config import Configurator
from pathlib import Path
import os
import time

file_path = Path(__file__)
yaml_fp = os.path.join(str(file_path.parent.parent), 'configs', 'jconf_1.yaml')
    
configurator = Configurator(yaml_fp)
config = configurator.configurate()

path_gen = PathGenerator(config, build=False)



graphs = Graphs()
g = graphs.get_graph(complexity=7)

start = list(g.start)
end = list(g.end)

xx,xy,uv,uomega,_ = path_gen.run(g, start, end)

fig, ax = plt.subplots(2,1)
fig.set_figheight(6)
fig.set_figwidth(10)

vel_ax = ax[0]
path_gen.mpc_generator.plot_vel(vel_ax, uv)
#vel_ax.set_xlabel('Time [s]')
vel_ax.set_ylabel('Linear elocity [m/s]')
vel_ax.legend(['Linear velocity'])
vel_ax.grid('on')

omega_ax = ax[1]
path_gen.mpc_generator.plot_omega(omega_ax, uomega)
omega_ax.set_xlabel('Time [s]')
omega_ax.set_ylabel('Angular velocity [rad/s]')
omega_ax.legend(['Angular velocity'])
omega_ax.grid('on')

fig2, ax2 = plt.subplots(1,1)
fig2.set_figheight(10)
fig2.set_figwidth(10)
path_ax =  ax2
path_gen.ppp.plot_all(path_ax)
path_ax.plot(xx, xy, c='b', label='Path', marker = 'o', alpha=0.5, markersize = 3)
path_ax.plot(start[0], start[1], marker='*', color='g', markersize = 15)
path_ax.plot(end[0], end[1], marker='*', color='r', markersize = 15)

path_ax.set_xlabel('X [m]', fontsize = 20)
path_ax.set_ylabel('Y [m]', fontsize = 20)

legend_elems = [    Line2D([0], [0], color='k', label='Original Boundary' ),
                    Line2D([0], [0], color='g', label='Padded Boundary'),
                    Line2D([0], [0], color='r', label='Original Obstacles' ),
                    Line2D([0], [0], color='y', label='Padded Obstacles' ),
                    Line2D([0], [0], linestyle='--', color='k', label='A-Star Path' ),
                    Line2D([0], [0], marker='o', color='b', label='Generated Path', alpha = 0.5),
                    Line2D([0], [0], color='g', marker='*', label='Start Position'),
                    Line2D([0], [0], color='r', marker='*', label='End Position'),
                    ]

path_ax.legend(handles = legend_elems, fontsize=15)
path_ax.axis('equal')

plt.show()

plt.close('all')