import opengen as og
import casadi.casadi as cs
import matplotlib.pyplot as plt
import numpy as np
import math

MAX_NUMBER_OF_OBSTACLES = 10
MAX_SOVLER_TIME = 10_000_000

(nu, nz, N) = (2, 6, 70) # v and w decision, 3 init and 3 end positions as parameter, 30 time steps
nedges = 4
(nobs, Nobs) = (2+2*nedges, MAX_NUMBER_OF_OBSTACLES) #x,y, 4 phis and 4 psis
ts = 0.2
(q, qtheta, rv, rw, qN, qthetaN) = (0.1, 0, 0.1, 0.1, 2000, 0)

class MpcModule:

    def __init__(self, N, nu, nz, nobs, Nobs, nedges, ts):
        self.N_lookahead = N    # lookahead
        self.nu = nu            # decision per time steps
        self.nz = nz            # number of states
        self.nobs = nobs        # number of params per obstacle
        self.Nobs = Nobs        # number of obstacles
        self.nedges = nedges    # number of edges per obstacle
        self.rectangles = []    # list of rectangle parameters
        self.ts = ts            # time step size
        

    def build(self):
        # Build parametric optimizer
        # ------------------------------------
        
        u = cs.SX.sym('u', self.nu*self.N_lookahead)
        z0 = cs.SX.sym('z0', self.nz+self.Nobs*self.nobs+1) #init + final position, obstacle params, number of obstacles to consider


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


            for i in range(0, self.Nobs*self.nobs, self.nobs): # LOOP OVER ALL OBSTACLES
                obstacle = z0[self.nz+i:self.nz+i+self.nobs]
                xi, yi = obstacle[0], obstacle[1] # Center of obstacle i
                xdiff = x-xi
                ydiff = y-yi
                angle = cs.fmod(cs.atan2(ydiff, xdiff),2.0*cs.pi) # angle between obstacle and current position
                
                phi_sum_nearest = cs.SX.zeros(1,1)
                smallest_angle_diff = cs.pi*2.0
                
                psi_nearest = cs.SX.zeros(1,1)
                phi_sum = 0

                for j in range(0,self.nedges): # LOOP OVER ALL EDGES IN OBSTACLE i

                    psi = obstacle[2+self.nedges+j] # distance for current edge
                    
                    phi_sum += obstacle[2+j] # acccum angle for current edge
                    temp_diff = cs.fmin((2 * cs.pi) - cs.fabs(phi_sum - angle), cs.fabs(phi_sum - angle))
                    temp_bool = temp_diff < smallest_angle_diff
                    psi_nearest = cs.if_else(temp_bool, psi, psi_nearest)
                    phi_sum_nearest = cs.if_else(temp_bool, phi_sum, phi_sum_nearest)
                    smallest_angle_diff = cs.if_else(temp_bool, temp_diff, smallest_angle_diff)

                Fi = xdiff*cs.cos(phi_sum_nearest) + ydiff*cs.sin(phi_sum_nearest)
                '''is_inside = cs.if_else(Fi < psi_nearest, True, False)

                psi = cs.if_else(is_inside, psi_nearest, 0)
                d  = cs.if_else(is_inside, Fi, 0)
                c += cs.if_else(cs.logic_and(is_inside, i < z0[-1]), cs.fmax(0,psi-d), 0)'''
                c += cs.if_else(i < z0[-1], cs.fmax(0,psi_nearest-Fi), 0)


        cost += qN*((x-xref)**2 + (y-yref)**2) + qthetaN*(theta-thetaref)**2

        umin = [-1.5] * (nu*N) 
        umax = [1.5] * (nu*N) 
        bounds = og.constraints.Rectangle(umin, umax)

        problem = og.builder.Problem(u, z0, cost).with_aug_lagrangian_constraints(c, og.constraints.Zero(), og.constraints.Zero()) \
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

    def x_y_from_rect(self, rect): 
        return (rect[0]-rect[-1], rect[0]+rect[-1], rect[1]-rect[-1], rect[1]+rect[-1])

    def plot_trajectory(self, u, x_init):
        nx = self.nz//2 # nx is start and end state
        x_states = [0.0] * (nx*(self.N_lookahead+2))
        x_states[0:nx+1] = x_init
        for t in range(0, self.N_lookahead):
            u_t = u[t*nu:(t+1)*nu]

            x = x_states[t * nx]
            y = x_states[t * nx + 1]
            theta = x_states[t * nx + 2]

            theta_dot = u_t[1]

            x_states[(t+1)*nx] = x + ts * (u_t[0] * cs.cos(theta))
            x_states[(t+1)*nx+1] = y + ts * (u_t[0] * cs.sin(theta))
            x_states[(t+1)*nx+2] = theta + ts*theta_dot

        xx = x_states[0:nx*N:nx]
        xy = x_states[1:nx*N:nx]

        plt.subplot(313)
        plt.plot(xx, xy, '-o')
        for rect in self.rectangles:
            xmin, xmax, ymin, ymax = self.x_y_from_rect(rect)
            plt.plot([xmin,xmax,xmax,xmin, xmin], [ymin, ymin, ymax, ymax, ymin])
        
        plt.axis('equal')

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
        x_finish = [2, 2, math.pi/4]
        initial_guess = [1.5, 0.1] * 7 + [1.5, -cs.pi/4/0.2/3] * 3 + [1.5, 0.0] * 4 + [0.5, cs.pi/4/0.2/3] * 6 + [1.5, 0] * 7
        initial_guess += (N*nu-len(initial_guess))*[0.0]
        #plot_trajectory(inital_guess, x_init)

        mng = og.tcp.OptimizerTcpManager('python_test_build/navigation')
        mng.start()

        mng.ping()

        rect1 = [1, 1, 0, cs.pi/2, cs.pi/2, cs.pi/2, 0.75, 0.75, 0.75, 0.75]
        self.rectangles.append(rect1)

        rect2 = [0, -1.5, 0, cs.pi/2, cs.pi/2, cs.pi/2, 0.75, 0.75, 0.75, 0.75]
        self.rectangles.append(rect2)

        rectangles = [0.0] * (Nobs*nobs)
        for i, rect in enumerate(self.rectangles):
            rectangles[i*nobs:(i+1)*nobs] = rect

        parameters = x_init+x_finish+rectangles+[3]
        
        solution = mng.call(parameters, initial_guess=initial_guess)
        mng.kill()

        print(f'Solution time: {solution["solve_time_ms"]}')
        print(f'Exit status: {solution["exit_status"]}')


        # Plot solution
        # ------------------------------------
        
        u_star = solution['solution']
        self.plot_vel_omega(u_star)

        # Plot trajectory
        # ------------------------------------
        self.plot_trajectory(u_star, x_init)

        plt.show()

if __name__ == '__main__':
    do_build = True
    do_run = True

    mpc_module = MpcModule(N, nu, nz, nobs, Nobs, nedges, ts)

    if do_build:
        response = input('Are you sure you want to build? (type anything)')
        if response:
            mpc_module.build()

    if do_run:
        mpc_module.run()