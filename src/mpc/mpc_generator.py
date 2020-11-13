import opengen as og
import casadi.casadi as cs
import matplotlib.pyplot as plt
import numpy as np
import math
import time

MAX_NUMBER_OF_OBSTACLES = 10
MAX_SOVLER_TIME = 10_000_000

class Config:
    def __init__(self):
        self.N_hor = 10                    # lookahead
        self.nu = 2                            # decision per time steps
        self.nz = 6                            # number of states
        self.nobs = 3                        # number of params per obstacle
        self.Nobs =  10                       # number of obstacles
        self.ts = 0.2                            # time step size
        self.vmax = 1.5
        self.omega_max = 0.5
        self.q = 1
        self.qtheta = 1
        self.rv = 10
        self.rw = 10
        self.qN = 200
        self.qthetaN = 10

class MpcModule:

    def __init__(self, config):
        self.obstacles = []                     # list of obstacles
        self.config = config

    def rough_ref(self, pos, node_list, i=0):
    
        #i = 0 # current index of target node
        v = 0.9*self.config.vmax # Want to plan reference trajectory with less than top speed 
        # so that there is room for the mpc solution tocatch up since it will likely 
        # have a slightly longer trajectory
        x_ref = []
        y_ref = []
        theta_ref = []
        x,y = pos
        x_target, y_target = node_list[i]
        
        traveling = True
        while(traveling):# for n in range(N):
            t = self.config.ts
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
    
    def build(self):
        # Build parametric optimizer
        # ------------------------------------
        
        u = cs.SX.sym('u', self.config.nu*self.config.N_hor)
        z0 = cs.SX.sym('z0', self.config.nz+self.config.Nobs*self.config.nobs) #init + final position, obstacle params, circle radius

        (x, y, theta) = (z0[0], z0[1], z0[2])
        (xref , yref, thetaref) = (z0[3], z0[4], z0[5])
        cost = 0
        c = 0

        for t in range(0, self.config.nu*self.config.N_hor, self.config.nu): # LOOP OVER TIME STEPS
            cost += self.config.q*((x-xref)**2 + (y-yref)**2) + self.config.qtheta*(theta-thetaref)**2
            u_t = u[t:t+self.config.nu]
            cost += self.config.rv * u_t[0]**2 + self.config.rw * u_t[1] ** 2
            x += self.config.ts * (u_t[0] * cs.cos(theta))
            y += self.config.ts * (u_t[0] * cs.sin(theta))
            theta += self.config.ts * u_t[1]

            xs = z0[self.config.nz:self.config.nz+self.config.Nobs*self.config.nobs:self.config.nobs]
            ys = z0[self.config.nz+1:self.config.nz+self.config.Nobs*self.config.nobs:self.config.nobs]
            rs = z0[self.config.nz+2:self.config.nz+self.config.Nobs*self.config.nobs:self.config.nobs]

            xdiff = x-xs
            ydiff = y-ys

            c+= cs.fmax(0, rs**2-xdiff**2-ydiff**2)

        cost += self.config.qN*((x-xref)**2 + (y-yref)**2) + self.config.qthetaN*(theta-thetaref)**2

        umin = [-self.config.vmax,-self.config.omega_max] * self.config.N_hor 
        umax = [self.config.vmax, self.config.omega_max] * self.config.N_hor  
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

    def plot_trajectory(self, u, x_init):
        nx = self.config.nz//2 # nz is start and end state
        states = [0.0] * (nx*(self.config.N_hor+1))
        states[0:nx] = x_init
        for t in range(0, self.config.N_hor):
            u_t = u[t*nu:(t+1)*nu]

            x = states[t * nx]
            y = states[t * nx + 1]
            theta = states[t * nx + 2]

            theta_dot = u_t[1]

            states[(t+1)*nx] = x + ts * (u_t[0] * cs.cos(theta))
            states[(t+1)*nx+1] = y + ts * (u_t[0] * cs.sin(theta))
            states[(t+1)*nx+2] = theta + ts*theta_dot

        xx = states[0:nx*N:nx]
        xy = states[1:nx*N:nx]

        plt.subplot(313)
        plt.plot(xx, xy, '-o')
        self.plot_obstacles()
        
        plt.axis('equal')
        plt.grid('on')

    def plot_obstacles(self):
        for circ in self.obstacles:
            circle = plt.Circle((circ[0],circ[1]),circ[2],color='r')
            plt.gcf().gca().add_artist(circle)

    def plot_vel_omega(self, u_star):
        time = np.arange(0, self.config.ts*self.config.N_hor, self.config.ts)
        
        vel = u_star[0:self.config.nu*self.config.N_hor:2]
        omega = u_star[1:self.config.nu*self.config.N_hor:2]

        plt.subplot(311)
        plt.plot(time, vel, '-o')
        plt.ylabel('velocity')
        plt.subplot(312)
        plt.plot(time, omega, '-o')
        plt.ylabel('angular velocity')
        plt.xlabel('Time')
    
    def run(self,x_init, x_finish, node_list, circles, radius):
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
        # Use TCP server
        # ------------------------------------
        
        x_ref,y_ref, theta_ref = self.rough_ref((x_init[0],x_init[1]), node_list)
        
        mng = og.tcp.OptimizerTcpManager('python_test_build/navigation')
        mng.start()
        mng.ping()

        tt = time.time()
        # take this amount of steps
        #for t in range(0,len(x_ref),take_steps):     
        system_input = []    
        terminal = False
        
        
        t=0
        states = x_init
        while(not terminal):   
            
            #x_init = [x_ref[t], y_ref[t], theta_ref[t]]
            x_init = states[-3:] # picks out current state for new initial state to solver
            if(len(x_ref)-1 <= t+self.config.N_hor): 
                take_steps = self.config.N_hor
                terminal = True
                x_finish = [x_ref[-1], y_ref[-1], theta_ref[-1]]
            else:
                take_steps = 5
                x_finish = [x_ref[t+self.config.N_hor], y_ref[t+self.config.N_hor],
                            theta_ref[t+self.config.N_hor]]
                
            parameters = x_init+x_finish+circles
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
            
            
        
            nx = self.config.nz//2 # nz is start and end state

            for i in range(take_steps):
                u_v = u[i*self.config.nu]
                u_omega = u[1+i*self.config.nu]
                
                system_input.append(u_v)
                system_input.append(u_omega)
                
                
                x = states[-3]
                y = states[-2]
                theta = states[-1]

                states.append(x + self.config.ts * (u_v * cs.cos(theta)))
                states.append(y + self.config.ts * (u_v * cs.sin(theta)))
                states.append(theta + self.config.ts*u_omega)
                
                t = t+1 

        
        mng.kill()
        
        total_time = int(1000*(time.time()-tt))   
        print("Total solution time: {} ms".format(total_time))
        #print(f'Solution time: {solution["solve_time_ms"]}') # cant use this for receeding horizon
        print(f'Exit status: {solution["exit_status"]}')


        # Plot solution
        # ------------------------------------
        
        xx = states[0:len(states):nx]
        xy = states[1:len(states):nx]
        uv = system_input[0:len(system_input):2]
        uomega = system_input[1:len(system_input):2]
        
        plt.plot(xx, xy, c='b', label='Path', marker = 'o', alpha =0.5)
        plt.plot(x_ref, y_ref, c='red', linewidth=2 ,label='reference_path')
        plt.axis('equal')
        plt.grid('on')
        plt.legend()
        
        #plt.plot.figure()
        #plt.plot(uv, c='b', label='velocity')
        #plt.legend()
        plt.show()    
        
        return xx,xy,uv,uomega    # uncomment if we want to return the traj

    def get_values_test1(self):    
        # Use TCP server
        # ------------------------------------
        x_init = [-2.0, -2.0, math.pi/4]
        x_finish = [15, 15, math.pi/4]

        # ToDo add node_list in the input argument instead
        node_list = [(0,0),(3,3),(5,3),(8,1),(x_finish[0],x_finish[1])]

        x_ref,y_ref, theta_ref = self.rough_ref((x_init[0],x_init[1]), node_list)  
        radius = 0.1

        self.obstacles = [[1, 1, radius],[0, -1, radius], [2, -1, radius],
                            [2, 1, radius],[-0.5, -0.2, radius]]

        circles = [0.0] * (self.config.Nobs*self.config.nobs)
        for i, circ in enumerate(self.obstacles):
            circles[i*self.config.nobs:(i+1)*self.config.nobs] = circ[0:self.config.nobs]
            
        return x_init, x_finish, node_list, circles, radius


if __name__ == '__main__':
    do_build = False
    do_run = True
    
    config = Config()
    mpc_module = MpcModule(config)


    if do_build:
        mpc_module.build()

    if do_run:
        # Run with test case
        x_init, x_finish, node_list, circles, radius = mpc_module.get_values_test1()
        xx,xy,uv,uomega = mpc_module.run(x_init, x_finish, node_list, circles, radius)
        