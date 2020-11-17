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
        ax.plot(x_ref, y_ref, label='Rough ref')

        
        mng = og.tcp.OptimizerTcpManager(self.config.build_directory + os.sep + self.config.optimizer_name)
        mng.start()
        mng.ping()
  
        terminal = False
        t=0
        total_solver_time = 0

        system_input = []  
        states = start
        ref_points = [(x,y) for x,y in zip(x_ref, y_ref)]
        try:
            while (not terminal) and t < 10000:  
                
                x_init = states[-3:] # picks out current state for new initial state to solver
                
                # Create constraints from verticies 
                constraint_origin = self.ppp.find_closest_vertices((x_init[0], x_init[1]), self.config.Nobs, 0)
                constraints = [0.0] * self.config.Nobs*self.config.nobs
                for i, origin in enumerate(constraint_origin):
                    constraints[i*self.config.nobs:(i+1)*self.config.nobs] = list(origin) + [self.config.vehicle_width/2 + self.config.vehicle_margin]

                # Take out final reference point
                _, idx = self.ppp.get_closest_vert((x_init[0], x_init[1]), ref_points)
                if (idx+self.config.N_hor >= len(x_ref)): 
                    take_steps = self.config.N_hor
                    x_finish = [x_ref[-1], y_ref[-1], theta_ref[-1]]
                    tmp = min(len(x_ref)-1, idx)
                    tmpx = x_ref[tmp:] + [end[0]] * (self.config.N_hor - (len(x_ref)-tmp))
                    tmpy = y_ref[tmp:] + [end[1]] * (self.config.N_hor - (len(y_ref)-tmp))
                    tmpt = theta_ref[tmp:] + [end[2]] * (self.config.N_hor - (len(theta_ref)-tmp))
                    
                else:
                    take_steps = 7
                    x_finish = [x_ref[idx+self.config.N_hor],
                                y_ref[idx+self.config.N_hor],
                                theta_ref[idx+self.config.N_hor]]

                    tmpx = x_ref[idx:idx+self.config.N_hor]
                    tmpy = y_ref[idx:idx+self.config.N_hor]
                    tmpt = theta_ref[idx:idx+self.config.N_hor]
                
                 

                refs = [0.0] * (self.config.N_hor * self.config.nx)
                refs[0::self.config.nx] = tmpx
                refs[1::self.config.nx] = tmpy
                refs[2::self.config.nx] = tmpt
            
                parameters = x_init+x_finish+constraints+refs
                try:
                    exit_status, solver_time = self.mpc_generator.run(parameters, mng, take_steps, system_input, states)
                except RuntimeError as err:
                    print(err)
                    mng.kill()
                    return

                if exit_status in self.config.bad_exit_codes:
                    print(f"[MPC] Bad converge status: {exit_status}")
                    ax.plot(states[0:-1:3], states[1:-1:3])
                    #plt.show()
                
                t += take_steps
                total_solver_time += solver_time

                if np.allclose(states[-3:-1],end[0:2],atol=0.01,rtol=0):
                    terminal = True
                
        except KeyboardInterrupt:
            print("[MPC] killing TCP connection to MCP solver...")
            mng.kill()
            return
            

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
        plt.show()
        
        plt.figure()
        plt.plot(uv, c='b', label='velocity')
        plt.plot(uomega, c='r', label='omega')
        plt.legend()
        plt.show()    
        
        return xx,xy,uv,uomega    # uncomment if we want to return the traj