# -*- coding: utf-8 -*-
"""
Created on Tue Nov 10 20:17:38 2020

@author: anton
"""

import opengen as og
import casadi.casadi as cs
import matplotlib.pyplot as plt
import numpy as np
import math
import time


def rough_ref(pos, node_list,t_s,vmax, i=0):
    
  
    #i = 0 # current index of target node
    v = 0.9*vmax # Want to plan reference trajectory with less than top speed 
    # so that there is room for the mpc solution tocatch up since it will likely 
    # have a slightly longer trajectory
    x_ref = []
    y_ref = []
    theta_ref = []
    x,y = pos
    x_target, y_target = node_list[i]
    
    traveling = True
    while(traveling):# for n in range(N):
        t = t_s
        while(t>0):
            if(math.hypot(x_target-x,y_target-y) == 0):
                traveling = False
                break
            x_dir = (x_target-x)/math.hypot(x_target-x,y_target-y)
            y_dir = (y_target-y)/math.hypot(x_target-x,y_target-y)
            dist = math.hypot(x_target-x,y_target-y)
            time = dist/v
            if time > t:
                # Travel along in the direction of the target for t
                x,y = x+x_dir*v*t, y+y_dir*v*t 
                t = 0 # set t to 0 to get out of the loop
            else:
                # Travel to the current target and then set a new target
                x,y = x+x_dir*v*time, y+y_dir*v*time
                t = t-time # update how much time you have left on the current time step
                i = i+1
                if i >len(node_list)-1 :
                    traveling = False
                    break
                else:
                    x_target, y_target = node_list[i] # set a new target node
               
        x_ref.append(x)
        y_ref.append(y)
        theta_ref.append(math.atan2(y_dir,x_dir))
        
    return x_ref, y_ref, theta_ref

class MpcModule:

    def __init__(self, N, nu, nz, nobs, Nobs, nedges, ts, obstacle_type):
        self.N_lookahead = N                    # lookahead
        self.nu = nu                            # decision per time steps
        self.nz = nz                            # number of states
        self.nobs = nobs                        # number of params per obstacle
        self.Nobs = Nobs                        # number of obstacles
        self.nedges = nedges                    # number of edges per obstacle
        self.obstacles = []                     # list of rectangle parameters
        self.ts = ts                            # time step size
        self.obstacle_type = obstacle_type      # obstacle type

    def build(self):
        # Build parametric optimizer
        # ------------------------------------
        
        u = cs.SX.sym('u', self.nu*self.N_lookahead)
        z0 = cs.SX.sym('z0', self.nz+self.Nobs*self.nobs+1) #init + final position, obstacle params, circle radius

        (x, y, theta) = (z0[0], z0[1], z0[2])
        (xref , yref, thetaref) = (z0[3], z0[4], z0[5])
        cost = 0
        c = 0

        for t in range(0, self.nu*self.N_lookahead, self.nu): # LOOP OVER TIME STEPS
            cost += q*((x-xref)**2 + (y-yref)**2) + qtheta*(theta-thetaref)**2
            u_t = u[t:t+nu]
            cost += rv * u_t[0]**2 + rw * u_t[1] ** 2
            x += ts * (u_t[0] * cs.cos(theta))
            y += ts * (u_t[0] * cs.sin(theta))
            theta += ts * u_t[1]

            xs = z0[self.nz:self.nz+self.Nobs*self.nobs:2]
            ys = z0[self.nz+1:self.nz+self.Nobs*self.nobs:2]

            xdiff = x-xs
            ydiff = y-ys

            circ_radius = z0[self.nz+self.Nobs*self.nobs]

            c+= cs.fmax(0, circ_radius**2-xdiff**2-ydiff**2)

        cost += qN*((x-xref)**2 + (y-yref)**2) + qthetaN*(theta-thetaref)**2

        umin = [-vmax,-omega_max] *N 
        umax = [vmax, omega_max] *N  
        bounds = og.constraints.Rectangle(umin, umax)

        problem = og.builder.Problem(u, z0, cost).with_penalty_constraints(c) \
                                                    .with_constraints(bounds)
        build_config = og.config.BuildConfiguration()\
            .with_build_directory("python_test_build")\
            .with_build_mode("debug")\
            .with_tcp_interface_config()

        meta = og.config.OptimizerMeta()\
            .with_optimizer_name("navigation")

        solver_config = og.config.SolverConfiguration()\
                .with_tolerance(1e-5)\
                .with_max_duration_micros(MAX_SOVLER_TIME)

        builder = og.builder.OpEnOptimizerBuilder(problem, 
                                                meta,
                                                build_config, 
                                                solver_config) \
            .with_verbosity_level(1)
        builder.build()

    def plot_obstacles(self):
        if self.obstacle_type == 'rect':
            for rect in self.obstacles:
                xmin, xmax, ymin, ymax = self.x_y_from_rect(rect)
                plt.plot([xmin,xmax,xmax,xmin, xmin], [ymin, ymin, ymax, ymax, ymin])

        elif self.obstacle_type == 'circ':
            for circ in self.obstacles:
                circle = plt.Circle((circ[0],circ[1]),circ[2],color='r')
                plt.gcf().gca().add_artist(circle)

    def plot_vel_omega(self, u_star):
        time = np.arange(0, self.ts*self.N_lookahead, self.ts)
        
        vel = u_star[0:nu*N:2]
        omega = u_star[1:nu*N:2]

        plt.subplot(311)
        plt.plot(time, vel, '-o')
        plt.ylabel('velocity')
        plt.subplot(312)
        plt.plot(time, omega, '-o')
        plt.ylabel('angular velocity')
        plt.xlabel('Time')
    
        
        
    def run(self):
        # Use TCP server
        # ------------------------------------
        x_init = [-2.0, -2.0, math.pi/4]

        #x_finish = [2.0, 2.0, math.pi/4]
        x_finish = [15, 15, math.pi/4]
        
        # ToDo add node_list in the input argument instead
        node_list = [(0,0),(3,3),(5,3),(8,1),(x_finish[0],x_finish[1])]
        
        x_ref,y_ref, theta_ref = rough_ref((x_init[0],x_init[1]), node_list,ts,1.5)
        
        radius = 0.1

        mng = og.tcp.OptimizerTcpManager('python_test_build/navigation')
        mng.start()

        mng.ping()
        
        # ToDo add obstacles from a list in the input argument
        circ = [1, 1, radius]
        self.obstacles.append(circ)

        circ = [0, -1, radius]
        self.obstacles.append(circ)

        circ = [2, -1, radius]
        self.obstacles.append(circ)

        circ = [2, 1, radius]
        self.obstacles.append(circ)

        circ = [-0.5, -0.2, radius]
        self.obstacles.append(circ)

        circles = [100.0] * (self.Nobs*self.nobs)
        for i, circ in enumerate(self.obstacles):
            circles[i*self.nobs:(i+1)*self.nobs] = circ[0:2]

        tt = time.time()
        # take this amount of steps
        #for t in range(0,len(x_ref),take_steps):
        terminal = False
        t=0
        states = x_init
        while(not terminal):   
            
            #x_init = [x_ref[t], y_ref[t], theta_ref[t]]
            x_init = states[-3:] # picks out current state for new initial state to solver
            if(len(x_ref)-1 <= t+self.N_lookahead): 

                take_steps = self.N_lookahead
                terminal = True
                x_finish = [x_ref[-1], y_ref[-1], theta_ref[-1]]
            else:
                take_steps = 5
                x_finish = [x_ref[t+self.N_lookahead], y_ref[t+self.N_lookahead],
                            theta_ref[t+self.N_lookahead]]
                
            parameters = x_init+x_finish+circles+[radius]
            solution = mng.call(parameters)
            
            
            if solution.is_ok():
                # Solver returned a solution
                solution_data = solution.get()
                u = solution_data.solution
                exit_status = solution_data.exit_status
                solver_time = solution_data.solve_time_ms
            else:
                # Invocation failed - an error report is returned
                solver_error = solution.get()
                error_code = solver_error.code
                error_msg = solver_error.message
                mng.kill() # kill so rust code wont keep running if python crashes
                print(error_msg)
                return
            
            
        
            nx = self.nz//2 # nz is start and end state

            for i in range(take_steps):
                u_v = u[i*nu]
                u_omega = u[1+i*nu]
                
                
                x = states[-3]
                y = states[-2]
                theta = states[-1]

                states.append(x + ts * (u_v * cs.cos(theta)))
                states.append(y + ts * (u_v * cs.sin(theta)))
                states.append(theta + ts*u_omega)
                
                t = t+1 

        
        mng.kill()
        
        total_time = time.time()-tt   
        print("Total solution time: {}".format(total_time))
        #print(f'Solution time: {solution["solve_time_ms"]}') # cant use this for receeding horizon
        print(f'Exit status: {solution["exit_status"]}')


        # Plot solution
        # ------------------------------------
        
        xx = states[0:len(states):nx]
        xy = states[1:len(states):nx]
        
        
        plt.plot(xx, xy, c='b', label='Path', marker = 'o', alpha =0.5)
        
        plt.plot(x_ref, y_ref, c='red', linewidth=2 ,label='reference_path')
        
        plt.axis('equal')
        plt.grid('on')
        plt.legend()
        plt.show()        
        
        

MAX_NUMBER_OF_OBSTACLES = 10
MAX_SOVLER_TIME = 10_000_000
(vmax, omega_max) = (1.5,0.2)

if __name__ == '__main__':
    do_build = False
    do_run = True

    
    
    (nu, nz, N) = (2, 6, 10) # v and w decision, 3 init and 3 end positions as parameter, 30 time steps
    nedges = None
    (nobs, Nobs) = (2, MAX_NUMBER_OF_OBSTACLES) # x,y for each circle
    ts = 0.2
    (q, qtheta, rv, rw, qN, qthetaN) = (1, 1, 1, 1, 100, 10)
    obstacle_type = 'circ2'
    
    # 
    mpc_module = MpcModule(N, nu, nz, nobs, Nobs, nedges, ts, obstacle_type)

    if do_build:
        response = input('Are you sure you want to build? (type anything)')
        if response:
            mpc_module.build()

    if do_run:
        mpc_module.run()
        
    

    