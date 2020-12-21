from visibility.visibility import PathPreProcessor
from mpc.mpc_generator import MpcModule
import opengen as og
import os
import numpy as np
import time
from matplotlib.gridspec import GridSpec
from matplotlib.lines import Line2D
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import math
import itertools

class PathGenerator:
    """ Class responsible for generating a smooth trajectory based on inital preproccsing 
        together with mpc-solver. Uses a configuration specified in utils/config.py
    """
    def __init__(self, config, build=False, verbose=False):
        self.config = config 
        self.verbose = verbose
        self.ppp = PathPreProcessor(self.config)
        self.mpc_generator = MpcModule(self.config)
        self.time_dict = dict()
        
        if build:
            self.mpc_generator.build()

    def plot_results(self, xx, xy, vel, omega, start, end, dynamic = False):
        if dynamic:
            self.plot_dynamic_results(xx, xy, vel, omega, start, end)
        else:
            self.plot_static_results(xx, xy, vel, omega, start, end)

    def plot_static_results(self, xx, xy, vel, omega, start, end):
        fig = plt.figure(constrained_layout = True)
        gs = GridSpec(2, 4, figure=fig)

        vel_ax = fig.add_subplot(gs[0,:2])
        self.mpc_generator.plot_vel(vel_ax, vel)
        vel_ax.set_xlabel('Time [s]')
        vel_ax.set_ylabel('Velocity [m/s]')

        omega_ax = fig.add_subplot(gs[1,:2])
        self.mpc_generator.plot_omega(omega_ax, omega)
        omega_ax.set_xlabel('Time [s]')
        omega_ax.set_ylabel('Angular velocity [rad/s]')

        path_ax =  fig.add_subplot(gs[:,2:])
        self.ppp.plot_all(path_ax)
        path_ax.plot(xx, xy, c='b', label='Path', marker = 'o', alpha=0.5)
        path_ax.plot(start[0], start[1], marker='*', color='g', markersize = 15)
        path_ax.plot(end[0], end[1], marker='*', color='r', markersize = 15)

        path_ax.set_xlabel('X [m]', fontsize = 15)
        path_ax.set_ylabel('Y [m]', fontsize = 15)

        legend_elems = [    Line2D([0], [0], color='k', label='Original Boundary' ),
                            Line2D([0], [0], color='g', label='Padded Boundary'),
                            Line2D([0], [0], color='r', label='Original Obstacles' ),
                            Line2D([0], [0], color='y', label='Padded Obstacles' ),
                            Line2D([0], [0], marker='o', color='b', label='Generated Path', alpha = 0.5),
                            Line2D([0], [0], marker='*', color='g', label='Start Position', alpha = 0.5),
                            Line2D([0], [0], marker='*', color='r', label='End Position'),
                            ]
        
        path_ax.legend(handles = legend_elems)
        path_ax.axis('equal')

    def plot_dynamic_results(self, xx, xy, vel, omega, start, end):
        fig = plt.figure(constrained_layout = True)
        gs = GridSpec(2, 4, figure=fig)

        vel_ax = fig.add_subplot(gs[0,:2])
        vel_line, = vel_ax.plot([1], '-o', markersize = 4, linewidth=2)
        vel_ax.set_xlabel('Time [s]')
        vel_ax.set_ylabel('Velocity [m/s]')
        vel_ax.set_ylim(-self.config.lin_vel_max - 0.1, self.config.lin_vel_max + 0.1)
        vel_ax.set_xlim(0, self.config.ts * len(xx))
        vel_ax.grid('on')

        omega_ax = fig.add_subplot(gs[1,:2])
        omega_line, = omega_ax.plot([1], '-o', markersize = 4, linewidth=2)
        omega_ax.set_ylim(-self.config.ang_vel_max - 0.1, self.config.ang_vel_max + 0.1)
        omega_ax.set_xlim(0, self.config.ts * len(xx))
        omega_ax.grid('on')
        omega_ax.set_xlabel('Time [s]')
        omega_ax.set_ylabel('Angular velocity [rad/s]')

        path_ax =  fig.add_subplot(gs[:,2:])
        path_ax.plot(start[0], start[1], marker='*', color='g', markersize = 15, label='Start')
        path_ax.plot(end[0], end[1], marker='*', color='r', markersize = 15,label='End')
        self.ppp.plot_all(path_ax)
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
        obs = [object] * len(self.ppp.dyn_obs_list)
        obs_padded = [object] * len(self.ppp.dyn_obs_list)

        for i in range(len(xx)):
            time = np.linspace(0, self.config.ts*i, i)
            omega_line.set_data(time, omega[:i])
            vel_line.set_data(time, vel[:i])
            path_line.set_data(xx[:i], xy[:i])
            


            veh = plt.Circle((xx[i], xy[i]), self.config.vehicle_width/2, color = 'b', alpha = 0.7, label='Robot')
            path_ax.add_artist(veh)
            for j, obstacle in enumerate(self.ppp.dyn_obs_list):
                p1,p2,freq,obstacle_radius = obstacle
                pos = self.ppp.generate_obstacle(p1,p2, freq, i * self.config.ts)
                obs[j] = plt.Circle(pos, obstacle_radius, color = 'r', alpha = 1, label='Obstacle')
                obs_padded[j] = plt.Circle(pos, obstacle_radius + self.config.vehicle_width/2 + self.config.vehicle_margin, color = 'y', alpha = 0.7, label='Padded obstacle')
                path_ax.add_artist(obs_padded[j])
                path_ax.add_artist(obs[j])
            

            plt.draw()
            plt.pause(self.config.ts / 10)
            veh.remove()    
            for j in range(len(self.ppp.dyn_obs_list)):
                obs[j].remove()
                obs_padded[j].remove()
            

        plt.show()

    def run(self, graph_map, start, end):
        """
        Parameters
        ----------
        x_init : TYPE list
            DESCRIPTION. [x_start, y_start,theta_start]
        x_finish : TYPE list
            DESCRIPTION. [x_finish, y_finish,theta_finish]
        node_list : TYPE list of tuples
            DESCRIPTION. holds all nodes to visit not including initial condition
        circles : TYPE list of lists
            DESCRIPTION. holds circle coordinates and 
        radius : TYPE double
            DESCRIPTION. radius of constraint circles

        Returns
        -------
        None.

        """
        t_temp = time.time() # Used to check run time for specific functions
        mng = og.tcp.OptimizerTcpManager(self.config.build_directory + os.sep + self.config.optimizer_name)
        mng.start()
        mng.ping()
        self.time_dict["opt_launch"] = int(1000*(time.time()-t_temp))
        
        parameter_list = [self.config.q, self.config.qtheta, self.config.lin_vel_penalty, self.config.lin_acc_penalty, self.config.qN, self.config.qthetaN, self.config.cte_penalty, self.config.lin_acc_penalty, self.config.ang_acc_penalty]
        tt = time.time() # for entire run time
        if self.verbose:
            print("[MPC] Building visibility graph")
        t_temp = time.time()
        self.ppp.prepare(graph_map)  
        self.time_dict["prepare"] =int(1000*(time.time()-t_temp))
        if self.verbose:
            print("[MPC] Findig A* solution")
        t_temp = time.time()
        path, _ = self.ppp.get_initial_guess((start[0],start[1]), (end[0],end[1]))
        self.time_dict["initial_guess"] = int(1000*(time.time()-t_temp))
        if self.verbose:
            print("[MPC] Getting rough reference")
        t_temp = time.time()
        x_ref, y_ref, theta_ref = self.mpc_generator.rough_ref((start[0],start[1]), path[1:])
        self.time_dict["rough_ref"] = int(1000*(time.time()-t_temp))
        t_temp = time.time()
        #x_ref, y_ref, theta_ref = self.mpc_generator.rough_ref2(start,end, path)
        if self.verbose:
            print("[MPC] Rough reference was succedfully generated")
      
        terminal = False
        t=0
        total_solver_time = []

        system_input = []  
        states = start.copy()
        ref_points = [(x,y) for x,y in zip(x_ref, y_ref)]
        refs = [0.0] * (self.config.N_hor * self.config.nx)
        idx = 0
        constraints = [0.0] * self.config.Nobs*self.config.nobs
        dyn_constraints = [0.0] * self.config.Ndynobs*self.config.nobs*self.config.N_hor
        params_per_dyn_obs = self.config.N_hor*self.config.Ndynobs*self.config.nobs
        t_temp = time.time()
        try:
            while (not terminal) and t < 500.0/self.config.ts:  

                x_init = states[-self.config.nx:] # picks out current state for new initial state to solver
                if len(self.ppp.original_obstacle_list):
                # Create constraints from verticies 
                    constraint_origin = self.ppp.find_closest_vertices((x_init[0], x_init[1]), self.config.Nobs, 0)
                    for i, origin in enumerate(constraint_origin):
                        constraints[i*self.config.nobs:(i+1)*self.config.nobs] = list(origin) + [self.config.vehicle_width/2 + self.config.vehicle_margin]
                
                for i, dyn_obstacle in enumerate(self.ppp.get_dyn_obstacle(t*self.config.ts, self.config.N_hor)):
                    dyn_constraints[i*params_per_dyn_obs:(i+1)*params_per_dyn_obs] = list(itertools.chain(*dyn_obstacle))
                    
                # reduce search space for closest reference point TODO: how to select "5"?
                lb_idx = max(0,idx-5*self.config.num_steps_taken)
                ub_idx = min(len(ref_points), idx+5*self.config.num_steps_taken)
                _, idx = self.ppp.get_closest_vert((x_init[0], x_init[1]), ref_points[lb_idx:ub_idx])
                idx += lb_idx #idx in orignal list
                if (idx+self.config.N_hor >= len(x_ref)): 
                    x_finish = end
                    tmp = min(len(x_ref)-1, idx)
                    tmpx = x_ref[tmp:] + [end[0]] * (self.config.N_hor - (len(x_ref)-tmp))
                    tmpy = y_ref[tmp:] + [end[1]] * (self.config.N_hor - (len(y_ref)-tmp))
                    tmpt = theta_ref[tmp:] + [end[2]] * (self.config.N_hor - (len(theta_ref)-tmp))
                else:
                    x_finish = [x_ref[idx+self.config.N_hor],
                                y_ref[idx+self.config.N_hor],
                                theta_ref[idx+self.config.N_hor]]

                    tmpx = x_ref[idx:idx+self.config.N_hor]
                    tmpy = y_ref[idx:idx+self.config.N_hor]
                    tmpt = theta_ref[idx:idx+self.config.N_hor]
                
                 

                
                refs[0::self.config.nx] = tmpx
                refs[1::self.config.nx] = tmpy
                refs[2::self.config.nx] = tmpt

                if len(system_input):
                    last_u = system_input[-self.config.nu:]
                else:
                    last_u = [0.0] * self.config.nu

                parameters = x_init+last_u+x_finish+last_u+parameter_list+constraints+dyn_constraints+refs
                try:
                    exit_status, solver_time = self.mpc_generator.run(parameters, mng, self.config.num_steps_taken, system_input, states)
                except RuntimeError as err:
                    if self.verbose:
                        print(err)
                    mng.kill()
                    return

                if exit_status in self.config.bad_exit_codes:
                    if self.verbose:
                        print(f"[MPC] Bad converge status: {exit_status}")
                    '''ax.plot(states[0:-1:3], states[1:-1:3])
                    #plt.show()'''
                
                total_solver_time.append(solver_time)

                if np.allclose(states[-3:-1],end[0:2],atol=0.1,rtol=0):# and abs(states[-1]-end[-1])<0.5:
                    terminal = True
                    
                    print("[MPC] MPC solution found.")

                t += self.config.num_steps_taken
                
        except KeyboardInterrupt:
            if self.verbose:
                print("[MPC] killing TCP connection to MCP solver...")
            mng.kill()
            nx = self.config.nx
            xx = states[0:len(states):nx]
            xy = states[1:len(states):nx]
            uv = system_input[0:len(system_input):2]
            uomega = system_input[1:len(system_input):2]
            
            return xx,xy,uv,uomega 
            

        mng.kill()
        
        total_time = int(1000*(time.time()-tt))
        mpc_time = int(1000*(time.time()-t_temp))
        
        print("Total solution time: {} ms".format(total_time))
    
        print("Total MPC solver time: {} ms".format(sum(total_solver_time)))
        
        self.time_dict["mpc_time"] = mpc_time
        self.time_dict["solver_time"] = sum(total_solver_time)
        self.time_dict["total_time"] = total_time
        
        # Plot solution
        # ------------------------------------
        nx = self.config.nx
        xx = states[0:len(states):nx]
        xy = states[1:len(states):nx]
        uv = system_input[0:len(system_input):2]
        uomega = system_input[1:len(system_input):2]
        
        self.runtime_analysis()
        
        return xx,xy,uv,uomega,total_solver_time    # uncomment if we want to return the traj
    
    
    
    def runtime_analysis(self,file_name =''):
        
        #if len(self.time_dict):
         #   print('There are no time entries run program before runtime analysis...')
          #  return
        print('###############################')
        print('####### Runtime Analysis ######')
        print('###############################')
        print("Launching optimizer : {} ms".format(self.time_dict["opt_launch"]))
        print("Prepare visibility graph : {} ms".format(self.time_dict["prepare"]))
        print("Generate rough reference path : {} ms".format(self.time_dict["rough_ref"]))
        print("MPC loop  : {} ms".format(self.time_dict["mpc_time"]))
        print("MPC loop solver : {} ms".format(int(self.time_dict["solver_time"])))
        print("MPC loop estimated python overhead : {} ms".format(int(self.time_dict["mpc_time"]-self.time_dict["solver_time"])))
        print("Total : {} ms".format(self.time_dict["total_time"]))
        
        print('###############################')
        
    #ToDo add a write to file if a file_name has been provided
        
        
        
        
        
        
        
        