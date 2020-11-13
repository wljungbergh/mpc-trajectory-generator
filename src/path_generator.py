from visibility.visibility import PathPreProcessor
from mpc.mpc_generator import MpcModule
from utils.config import Config
import opengen as og
import os
import numpy as np
import time
import matplotlib.pyplot as plt
import math

class PathGenerator:
    """ Class responsible for generating a smooth trajectory based on inital preproccsing 
        together with mpc-solver. Uses a configuration specified in utils/config.py
    """
    def __init__(self, build=False):
        self.config = Config()
        self.ppp = PathPreProcessor(self.config)
        self.mpc_generator = MpcModule(self.config)
        if build:
            self.mpc_generator.build()

    
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
        tt = time.time()
        print("[MPC] Building visibility graph")
        self.ppp.prepare(graph_map)
        print("[MPC] Findig A* solution")
        path, _ = self.ppp.get_initial_guess((start[0],start[1]), (end[0],end[1]))
        print("[MPC] Getting rough reference")
        x_ref, y_ref, theta_ref = self.mpc_generator.rough_ref((start[0],start[1]), path[1:])

        fig, ax = plt.subplots()
        self.ppp.plot_all(ax)
        ax.plot(x_ref, y_ref)

        
        mng = og.tcp.OptimizerTcpManager(self.config.build_directory + os.sep + self.config.optimizer_name)
        mng.start()
        mng.ping()
  
        terminal = False
        t=0
        total_solver_time = 0

        system_input = []  
        states = start
        try:
            while(not terminal):   
                
                x_init = states[-3:] # picks out current state for new initial state to solver
                constraint_origin = self.ppp.find_closest_vertices((x_init[0], x_init[1]), self.config.Nobs, 0)
                constraints = [0.0] * self.config.Nobs*self.config.nobs
                for i, origin in enumerate(constraint_origin):
                    constraints[i*self.config.nobs:(i+1)*self.config.nobs] = list(origin) + [self.config.vehicle_width/2 + self.config.vehicle_margin]

                if(len(x_ref)-1 <= t+self.config.N_hor): 
                    take_steps = self.config.N_hor
                    x_finish = [x_ref[-1], y_ref[-1], theta_ref[-1]]
                else:
                    take_steps = 5
                    x_finish = [x_ref[t+self.config.N_hor], y_ref[t+self.config.N_hor],
                                theta_ref[t+self.config.N_hor]]
                    
                parameters = x_init+x_finish+constraints
                try:
                    exit_status, solver_time = self.mpc_generator.run(parameters, mng, take_steps, system_input, states)
                except RuntimeError:
                    return

                if exit_status in self.config.bad_exit_codes:
                    print(f"Bad converge status: {exit_status}")
                    ax.plot(states[0:-1:3], states[1:-1:3])
                    #plt.show()
                
                t += take_steps
                total_solver_time += solver_time

                if np.allclose(states[-3:],end,atol=0.1):
                    terminal = True
        except KeyboardInterrupt:
            print("[MPC] killing TCP connection to MCP solver...")
            mng.kill()
            

        mng.kill()
        
        total_time = int(1000*(time.time()-tt))
        print("Total solution time: {} ms".format(total_time))
        print("Total MPC solver time: {} ms".format(total_solver_time))


        # Plot solution
        # ------------------------------------
        nx = self.config.nz//2
        xx = states[0:len(states):nx]
        xy = states[1:len(states):nx]
        uv = system_input[0:len(system_input):2]
        uomega = system_input[1:len(system_input):2]
        
        ax.plot(xx, xy, c='b', label='Path', marker = 'o', alpha =0.5)
        ax.plot(x_ref, y_ref, c='red', linewidth=2 ,label='reference_path')
        plt.axis('equal')
        plt.grid('on')
        plt.legend()
        
        #plt.plot.figure()
        #plt.plot(uv, c='b', label='velocity')
        #plt.legend()
        plt.show()    
        
        return xx,xy,uv,uomega    # uncomment if we want to return the traj