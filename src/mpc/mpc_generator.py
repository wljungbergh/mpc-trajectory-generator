import opengen as og
import casadi.casadi as cs
import matplotlib.pyplot as plt
import numpy as np
import math
import time
import os

MAX_SOVLER_TIME = 500_000


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
        v = self.config.throttle_ratio*self.config.lin_vel_max  # Want to plan reference trajectory with less than top speed 
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
        x_dir = (x_target-x)/safe_norm(x_target-x,y_target-y)
        y_dir = (y_target-y)/safe_norm(x_target-x,y_target-y)
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
                        x_dir = (x_target-x)/safe_norm(x_target-x,y_target-y)
                        y_dir = (y_target-y)/safe_norm(x_target-x,y_target-y)
                        theta_target = end_pos[2]
                        turning = True
                        traveling = False
                    else:
                        x_target, y_target = node_list[i]
                        x_dir = (x_target-x)/safe_norm(x_target-x,y_target-y)
                        y_dir = (y_target-y)/safe_norm(x_target-x,y_target-y)
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
                        x_dir = (x_target-x)/safe_norm(x_target-x,y_target-y)
                        y_dir = (y_target-y)/safe_norm(x_target-x,y_target-y)
                        theta_target = end_pos[2]
                        turning = True
                        traveling= False
                        break
                    else:
                        x_target, y_target = node_list[i] # set a new target node
                        x_dir = (x_target-x)/safe_norm(x_target-x,y_target-y)
                        y_dir = (y_target-y)/safe_norm(x_target-x,y_target-y)
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
        v = self.config.throttle_ratio*self.config.lin_vel_max # Want to plan reference trajectory with less than top speed 
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
    
    def cost_fn(self, state_curr, state_ref, q, qtheta):
        dx = (state_curr[0] - state_ref[0])**2
        dy = (state_curr[1] - state_ref[1])**2
        dtheta = (state_curr[2] - state_ref[2])**2
        cost = q*(dx + dy) + qtheta*dtheta
        return cost
        
    def build(self):
        # Build parametric optimizer
        # ------------------------------------
        
        u = cs.SX.sym('u', self.config.nu*self.config.N_hor)
        z0 = cs.SX.sym('z0', self.config.nz + self.config.N_hor + self.config.Nobs*self.config.nobs + self.config.Ndynobs*self.config.nobs*self.config.N_hor + self.config.nx*self.config.N_hor) #init + final position + cost, obstacle params, circle radius
                            # params,         vel_ref each step, number obstacles x num params per obs, num dynamic obstacles X num param/obs X time steps,    refernce path for each time step
        (x, y, theta, vel_init, omega_init) = (z0[0], z0[1], z0[2], z0[3], z0[4])
        (xref , yref, thetaref, velref, omegaref) = (z0[5], z0[6], z0[7], z0[8], z0[9])
        (q, qv, qtheta, rv, rw, qN, qthetaN, qCTE, acc_penalty, omega_acc_penalty) = (z0[10], z0[11], z0[12], z0[13], z0[14], z0[15], z0[16], z0[17], z0[18], z0[19])
        cost = 0
        obstacle_constraints = 0
        # Index where reference points start
        base = self.config.nz+self.config.N_hor+self.config.Nobs*self.config.nobs+self.config.Ndynobs*self.config.nobs*self.config.N_hor

        for t in range(0, self.config.N_hor): # LOOP OVER TIME STEPS
            
            u_t = u[t*self.config.nu:(t+1)*self.config.nu]
            cost += rv * u_t[0]**2 + rw * u_t[1] ** 2 # Penalize control actions
            cost += qv*(u_t[0]-z0[self.config.nz+t])**2 # Cost for diff between velocity and reference velocity
            cost += self.cost_fn((x,y,theta),(xref,yref,thetaref),q,qtheta)
            
            x += self.config.ts * (u_t[0] * cs.cos(theta))
            y += self.config.ts * (u_t[0] * cs.sin(theta))
            theta += self.config.ts * u_t[1]
            
            
            xs_static = z0[self.config.nz+self.config.N_hor:self.config.nz+self.config.N_hor+self.config.Nobs*self.config.nobs:self.config.nobs]
            ys_static = z0[self.config.nz+self.config.N_hor+1:self.config.nz+self.config.N_hor+self.config.Nobs*self.config.nobs:self.config.nobs]
            rs_static = z0[self.config.nz+self.config.N_hor+2:self.config.nz+self.config.N_hor+self.config.Nobs*self.config.nobs:self.config.nobs]

            # ordering is x,y,r for obstacle 0 for N_hor timesteps, then x,y,r for obstalce 1 for N_hor timesteps etc.
            end_of_static_obs_idx = self.config.nz + self.config.N_hor +  self.config.Nobs*self.config.nobs
            end_of_dynamic_obs_idx = end_of_static_obs_idx + self.config.Ndynobs*self.config.nobs*self.config.N_hor
            xs_dynamic = z0[end_of_static_obs_idx+t*self.config.nobs:end_of_dynamic_obs_idx:self.config.nobs*self.config.N_hor]
            ys_dynamic = z0[end_of_static_obs_idx+t*self.config.nobs+1:end_of_dynamic_obs_idx:self.config.nobs*self.config.N_hor]
            rs_dynamic = z0[end_of_static_obs_idx+t*self.config.nobs+2:end_of_dynamic_obs_idx:self.config.nobs*self.config.N_hor]

            xs = cs.vertcat(xs_static,xs_dynamic)
            ys = cs.vertcat(ys_static,ys_dynamic)
            rs = cs.vertcat(rs_static,rs_dynamic)

            xdiff = x-xs
            ydiff = y-ys

            obstacle_constraints += cs.fmax(0, rs**2-xdiff**2-ydiff**2)
            # Ellipse parameterized according to https://math.stackexchange.com/questions/426150/what-is-the-general-equation-of-the-ellipse-that-is-not-in-the-origin-and-rotate
            # xs and ys are ellipse center points, xdiff is as before
            # x_radius and y_radius are radii in "x" and "y" directions
            # As are angles of ellipses (positive from x axis)
            # distance_inside_ellipse = 1 - (xdiff*cs.cos(As)+ydiff*cs.sin(As))**2 / (x_radius**2) - (xdiff*cs.sin(As)-ydiff*cs.cos(As))**2 / (y_radius)**2
            # obstacle_constraints += cs.fmax(0, distance_inside_ellipse)

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
                # append distance (is actually squared distance)
                distances = cs.horzcat(distances,temp_vec[0]**2+temp_vec[1]**2)

            cost += cs.mmin(distances[1:])*qCTE


        
        cost += qN*((x-xref)**2 + (y-yref)**2) + qthetaN*(theta-thetaref)**2

        # Max speeds 
        umin = [self.config.lin_vel_min, -self.config.ang_vel_max] * self.config.N_hor
        umax = [self.config.lin_vel_max, self.config.ang_vel_max] * self.config.N_hor
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
        acc_min = [self.config.lin_acc_min] * self.config.N_hor 
        omega_min = [-self.config.ang_acc_max] * self.config.N_hor
        acc_max = [self.config.lin_acc_max] * self.config.N_hor
        omega_max = [self.config.ang_acc_max] * self.config.N_hor
        acc_bounds = og.constraints.Rectangle(acc_min + omega_min, acc_max + omega_max)
        # Accelerations cost
        cost += cs.mtimes(acc.T,acc)*acc_penalty
        cost += cs.mtimes(omega_acc.T,omega_acc)*omega_acc_penalty

        problem = og.builder.Problem(u, z0, cost).with_penalty_constraints(obstacle_constraints) \
                                                    .with_constraints(bounds) \
                                                    .with_aug_lagrangian_constraints(acc_constraints, acc_bounds)
        build_config = og.config.BuildConfiguration()\
            .with_build_directory(self.config.build_directory)\
            .with_build_mode(self.config.build_type)\
            .with_tcp_interface_config()

        meta = og.config.OptimizerMeta()\
            .with_optimizer_name(self.config.optimizer_name)

        solver_config = og.config.SolverConfiguration()\
                .with_tolerance(1e-4)\
                .with_max_duration_micros(MAX_SOVLER_TIME)

        builder = og.builder.OpEnOptimizerBuilder(problem, 
                                                meta,
                                                build_config, 
                                                solver_config) \
            .with_verbosity_level(1)
        builder.build()

    def plot_vel(self, ax, vel):
        
        time = np.linspace(0, self.config.ts*(len(vel)), len(vel))
        ax.plot(time, vel, '-o', markersize = 4, linewidth=2)

    def plot_omega(self, ax, omega):
        time = np.linspace(0, self.config.ts*(len(omega)), len(omega))
        ax.plot(time, omega, '-o', markersize = 4, linewidth=2)

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
        
        system_input += u[:self.config.nu*take_steps]

        for i in range(take_steps):
            u_v = u[i*self.config.nu]
            u_omega = u[1+i*self.config.nu]
            
            x = states[-3]
            y = states[-2]
            theta = states[-1]

            states += [x + self.config.ts * (u_v * math.cos(theta)), 
                       y + self.config.ts * (u_v * math.sin(theta)), 
                       theta + self.config.ts*u_omega]
        
        return exit_status, solver_time

def safe_norm(x,y):
    dist = math.hypot(x,y)
    return 1e-16 if dist == 0 else dist

        