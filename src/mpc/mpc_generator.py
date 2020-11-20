import opengen as og
import casadi.casadi as cs
import matplotlib.pyplot as plt
import numpy as np
import math
import time
import os

MAX_NUMBER_OF_OBSTACLES = 10
MAX_SOVLER_TIME = 10_000_000


class MpcModule:

    def __init__(self, config):
        self.obstacles = []                     # list of obstacles
        self.config = config
        
    def rough_ref2(self,start_pos, end_pos , node_list,i=0):
    
    # If starting position is included in 
    # the nodelist we will 
    
        if((start_pos[0],start_pos[1]) == node_list[0]): 
            i = 1 #target node is not starting position
            
        if((end_pos[0],end_pos[1]) != node_list[-1]):
            node_list.append((end_pos[0],end_pos[1])) #adds end node if not included
    
        #i = 0 # current index of target node
        v = self.config.throttle_ratio*self.config.vmax  # Want to plan reference trajectory with less than top speed 
        omega = self.config.throttle_ratio*self.config.omega_max 
        # so that there is room for the mpc solution tocatch up since it will likely 
        # have a slightly longer trajectory
        x_ref = []
        y_ref = []
        theta_ref = []
        if len(start_pos) == 2:
            start_pos = (start_pos[0],start_pos[1],0)
        if len(end_pos) == 2:
            end_pos = (end_pos[0],end_pos[1],0)
        
        
        x,y,theta = start_pos
        x_target, y_target = node_list[i]
        x_dir = (x_target-x)/math.hypot(x_target-x,y_target-y)
        y_dir = (y_target-y)/math.hypot(x_target-x,y_target-y)
        theta_target = math.atan2(y_dir,x_dir)
    
        traveling = True
        turning = True
        while(traveling or turning):# for n in range(N):
            t = self.config.ts
            x_ref.append(x)
            y_ref.append(y)
            theta_ref.append(theta)
            while(t>0):
                if(math.hypot(x_target-x,y_target-y) == 0 and not turning):
                    i=i+1

                    if i >len(node_list) :
                        # when we have arrived in the end node the desired angle is taken
                        x_target, y_target = end_pos[0:2]
                        x_dir = (x_target-x)/math.hypot(x_target-x,y_target-y)
                        y_dir = (y_target-y)/math.hypot(x_target-x,y_target-y)
                        theta_target = end_pos[2]
                        turning = True
                        traveling = False
                    else:
                        x_target, y_target = node_list[i]
                        x_dir = (x_target-x)/math.hypot(x_target-x,y_target-y)
                        y_dir = (y_target-y)/math.hypot(x_target-x,y_target-y)
                        theta_target = math.atan2(y_dir,x_dir)
                        turning = True #always set turning true for new target node
            
                if turning:
                    theta_target = math.atan2(y_dir,x_dir)
                    t_task = abs(theta_target-theta)/omega
                    if t_task > t:
                        # rotate in a node
                        theta += t*np.sign(theta_target-theta)*omega 
                        break
                    else:
                        # rotate in node
                        theta += t_task*np.sign(theta_target-theta)*omega
                        t = t-t_task # update how much time you have left on the current time step               
                        turning = False
                    
        
                dist = math.hypot(x_target-x,y_target-y)
                if(dist == 0):
                    #print('Breaking because dist in rouch ref is 0 ')
                    break # we dont want to divide with 0 
                    
                t_task = dist/v
                if t_task > t:
                    # Travel along in the direction of the target for t
                    x,y = x+x_dir*v*t, y+y_dir*v*t 
                    break
                else:
                    # Travel to the current target and then set a new target
                    x,y = x+x_dir*v*t_task, y+y_dir*v*t_task
                    t = t-t_task # update how much time you have left on the current time step
                    i = i+1
                    turning = True
                    if i >len(node_list)-1 :
                        x_target,y_target = end_pos[0],end_pos[1]
                        x_dir = (x_target-x)/math.hypot(x_target-x,y_target-y)
                        y_dir = (y_target-y)/math.hypot(x_target-x,y_target-y)
                        theta_target = end_pos[2]
                        turning = True
                        traveling= False
                        break
                    else:
                        x_target, y_target = node_list[i] # set a new target node
                        x_dir = (x_target-x)/math.hypot(x_target-x,y_target-y)
                        y_dir = (y_target-y)/math.hypot(x_target-x,y_target-y)
                        theta_target = math.atan2(y_dir,x_dir)

        if(end_pos[0] != x or end_pos[1] != y or end_pos[2] != theta):
            x_ref.append(x)
            y_ref.append(y)
            theta_ref.append(theta)
        
        travel_time = self.config.ts*len(x_ref)
        print('The estimated travel time will be: {} seconds'.format(travel_time))
        return x_ref, y_ref, theta_ref

    def rough_ref(self, pos, node_list, i=0):
    
        #i = 0 # current index of target node
        v = self.config.throttle_ratio*self.config.vmax # Want to plan reference trajectory with less than top speed 
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
    
    def cost_fn(self, state_curr, state_ref):
        dx = (state_curr[0] - state_ref[0])**2
        dy = (state_curr[1] - state_ref[1])**2
        dtheta = (state_curr[2] - state_ref[2])**2
        cost = self.config.q*(dx + dy) + self.config.qtheta*dtheta
        return cost
        
    def build(self):
        # Build parametric optimizer
        # ------------------------------------
        
        u = cs.SX.sym('u', self.config.nu*self.config.N_hor)
        z0 = cs.SX.sym('z0', self.config.nz + self.config.Nobs*self.config.nobs + self.config.nx*self.config.N_hor) #init + final position, obstacle params, circle radius

        (x, y, theta, vel_init, omega_init) = (z0[0], z0[1], z0[2], z0[3], z0[4])
        (xref , yref, thetaref, velref, omegaref) = (z0[self.config.nz/2], z0[self.config.nz/2+1], z0[self.config.nz/2+2], z0[self.config.nz/2+3], z0[self.config.nz/2+4])
        cost = 0
        obstacle_constraints = 0
        # Index where reference points start
        base = self.config.nz+self.config.Nobs*self.config.nobs

        for t in range(0, self.config.N_hor): # LOOP OVER TIME STEPS
            
            u_t = u[t*self.config.nu:(t+1)*self.config.nu]
            cost += self.config.rv * u_t[0]**2 + self.config.rw * u_t[1] ** 2
            x += self.config.ts * (u_t[0] * cs.cos(theta))
            y += self.config.ts * (u_t[0] * cs.sin(theta))
            theta += self.config.ts * u_t[1]

            cost += self.cost_fn((x,y,theta),(xref,yref,thetaref))

            xs = z0[self.config.nz:self.config.nz+self.config.Nobs*self.config.nobs:self.config.nobs]
            ys = z0[self.config.nz+1:self.config.nz+self.config.Nobs*self.config.nobs:self.config.nobs]
            rs = z0[self.config.nz+2:self.config.nz+self.config.Nobs*self.config.nobs:self.config.nobs]

            xdiff = x-xs
            ydiff = y-ys

            obstacle_constraints += cs.fmax(0, rs**2-xdiff**2-ydiff**2)

            # our current point
            p = cs.vertcat(x,y)

            # Initialize list with CTE to all line segments
            # https://math.stackexchange.com/questions/330269/the-distance-from-a-point-to-a-line-segment
            distances = cs.SX.ones(1)
            s2 = cs.vertcat(z0[base], z0[base+1])
            for i in range(1, self.config.N_hor):
                # set start point as previous end point
                s1 = s2
                # new end point
                s2 = cs.vertcat(z0[base+i*self.config.nx], z0[base+i*self.config.nx+1])
                # line segment
                s2s1 = s2-s1
                # t_hat
                t_hat = cs.dot(p-s1,s2s1)/(s2s1[0]**2+s2s1[1]**2+1e-16)
                # limit t
                t_star = cs.fmin(cs.fmax(t_hat,0.0),1.0)
                # vector pointing from us to closest point
                temp_vec = s1 + t_star*s2s1 - p
                # append distance
                distances = cs.horzcat(distances,temp_vec[0]**2+temp_vec[1]**2)

            cost += cs.mmin(distances[1:])*self.config.qCTE


        
        cost += self.config.qN*((x-xref)**2 + (y-yref)**2) + self.config.qthetaN*(theta-thetaref)**2

        # Max speeds 
        umin = [-self.config.vmax,-self.config.omega_max] * self.config.N_hor 
        umax = [self.config.vmax, self.config.omega_max] * self.config.N_hor  
        bounds = og.constraints.Rectangle(umin, umax)

        # Acceleration bounds and cost
        # Selected velocities
        v = u[0::2]
        omega = u[1::2]
        # Accelerations
        acc = (v-cs.vertcat(vel_init, v[0:-1]))/self.config.ts
        omega_acc = (omega-cs.vertcat(omega_init, omega[0:-1]))/self.config.ts
        acc_constraints = cs.vertcat(acc, omega_acc)
        # Acceleration bounds
        acc_min = [self.config.acc_min] * self.config.N_hor 
        omega_min = [-self.config.omega_acc_max] * self.config.N_hor
        acc_max = [self.config.acc_max] * self.config.N_hor
        omega_max = [self.config.omega_acc_max] * self.config.N_hor
        acc_bounds = og.constraints.Rectangle(acc_min + omega_min, acc_max + omega_max)
        # Accelerations cost
        cost += cs.mtimes(acc.T,acc)*self.config.acc_penalty
        cost += cs.mtimes(omega_acc.T,omega_acc)*self.config.omega_acc_penalty

        problem = og.builder.Problem(u, z0, cost).with_penalty_constraints(obstacle_constraints) \
                                                    .with_constraints(bounds) \
                                                    .with_aug_lagrangian_constraints(acc_constraints, acc_bounds)
        build_config = og.config.BuildConfiguration()\
            .with_build_directory(self.config.build_directory)\
            .with_build_mode("debug")\
            .with_tcp_interface_config()

        meta = og.config.OptimizerMeta()\
            .with_optimizer_name(self.config.optimizer_name)

        solver_config = og.config.SolverConfiguration()\
                .with_tolerance(1e-5)\
                .with_max_duration_micros(MAX_SOVLER_TIME)

        builder = og.builder.OpEnOptimizerBuilder(problem, 
                                                meta,
                                                build_config, 
                                                solver_config) \
            .with_verbosity_level(1)
        builder.build()

    def plot_vel(self, ax, vel):
        time = np.arange(0, self.config.ts*len(vel), self.config.ts)
        ax.plot(time, vel, '-o')

    def plot_omega(self, ax, omega):
        time = np.arange(0, self.config.ts*len(omega), self.config.ts)
        ax.plot(time, omega, '-o')

    def run(self, parameters, mng, take_steps, system_input, states):

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
            raise RuntimeError(f"MPC Solver error: {error_msg}")

        for i in range(take_steps):
            u_v = u[i*self.config.nu]
            u_omega = u[1+i*self.config.nu]
            
            system_input.append(u_v)
            system_input.append(u_omega)
            
            
            x = states[-3]
            y = states[-2]
            theta = states[-1]

            states.append(x + self.config.ts * (u_v * math.cos(theta)))
            states.append(y + self.config.ts * (u_v * math.sin(theta)))
            states.append(theta + self.config.ts*u_omega)

        return exit_status, solver_time

    def get_values_test1(self):    
        # Use TCP server
        # ------------------------------------
        x_init = [-2.0, -2.0, math.pi/4]
        x_finish = [15, 15, math.pi/4]

        # ToDo add node_list in the input argument instead
        node_list = [(0,0),(3,3),(5,3),(8,1),(x_finish[0],x_finish[1])]
 
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
        