import opengen as og
import casadi.casadi as cs
import matplotlib.pyplot as plt
import numpy as np
import math

MAX_NUMBER_OF_OBSTACLES = 10

(nu, nz, N) = (2, 6, 30) # v and w decision, 3 init and 3 end positions as parameter, 20 time steps
nedges = 4
(nobs, Nobs) = (2+2*nedges, MAX_NUMBER_OF_OBSTACLES) #x,y, 4 phis and 4 psis
ts = 0.1
(q, qtheta, r, qN, qthetaN) = (10, 0.1, 1, 200, 2)

def build():
    # Build parametric optimizer
    # ------------------------------------
    

    u = cs.SX.sym('u', nu*N)
    z0 = cs.SX.sym('z0', nz+Nobs*nobs+1)


    (x, y, theta) = (z0[0], z0[1], z0[2])
    (xref , yref, thetaref) = (z0[3], z0[4], z0[5])
    cost = 0
    c = 0

    for t in range(0, nu*N, nu): # LOOP OVER TIME STEPS
        cost += q*((x-xref)**2 + (y-yref)**2) + qtheta*(theta-thetaref)**2
        u_t = u[t:t+nu]
        cost += r * cs.dot(u_t, u_t)
        x += ts * (u_t[0] * cs.cos(theta))
        y += ts * (u_t[0] * cs.sin(theta))
        theta += ts * u_t[1]


        for i in range(0, Nobs*nobs, nobs): # LOOP OVER ALL OBSTACLES
            xi, yi = (z0[nz+i], z0[nz+i+1])
            xdiff = x-xi
            ydiff = y-yi
            angle = cs.fmod(cs.atan2(ydiff, xdiff),2.0*cs.pi)
            
            phi_sum_nearest = cs.SX.zeros(1,1)
            smallest_angle_diff = cs.pi*2.0
            
            psi_nearest = cs.SX.zeros(1,1)

            for j in range(0,nedges): # LOOP OVER ALL REGIONS IN OBSTACLE i

                psi = z0[nz+i+2+nedges+j]
                
                phi_sum = cs.dot(z0[nz+i+2:nz+i+2+j+1],cs.SX.ones(j+1,1))
                temp_diff = cs.fmod(phi_sum - angle, 2.0*cs.pi)
                temp_bool = temp_diff < smallest_angle_diff
                psi_nearest = cs.if_else(temp_bool, psi, psi_nearest)
                phi_sum_nearest = cs.if_else(temp_bool, phi_sum, phi_sum_nearest)
                smallest_angle_diff = cs.if_else(temp_bool, temp_diff, smallest_angle_diff)

                
            Fi = xdiff*cs.cos(phi_sum_nearest) + ydiff*cs.sin(phi_sum_nearest)
            is_inside = cs.if_else(Fi < psi_nearest, True, False)

            psi = cs.if_else(is_inside, psi_nearest, 0)
            d  = cs.if_else(is_inside, Fi, 0)
            c += cs.if_else(cs.logic_and(is_inside, i < z0[-1]), cs.fmax(0,psi-d), 0)


    cost += qN*((x-xref)**2 + (y-yref)**2) + qthetaN*(theta-thetaref)**2

    umin = [-1.4] * (nu*N) 
    umax = [1.4] * (nu*N) 
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
        .with_tolerance(1e-5)
    builder = og.builder.OpEnOptimizerBuilder(problem, 
                                            meta,
                                            build_config, 
                                            solver_config) \
        .with_verbosity_level(1)
    builder.build()

def run():
    # Use TCP server
    # ------------------------------------
    mng = og.tcp.OptimizerTcpManager('python_test_build/navigation')
    mng.start()

    mng.ping()

    x_init = [-2.0, -2.0, math.pi/4]
    x_finish = [2, 2, math.pi/4]

    xmin, xmax, ymin, ymax = (0.5, 1.5, 0.5, 1.5)
    rectangles = list(np.zeros(Nobs*nobs))
    rectangles[0:nobs] = [0, 0, 0, 1*2*cs.pi/4, 2*2*cs.pi/4, 3*2*cs.pi/4, 0.5, 0.5, 0.5, 0.5]
    xmin1, xmax1, ymin1, ymax1 = (-0.5, 0.5, -2, -1)
    #rectangles[nobs:2*nobs] = [0, -1.5, 0, 1*2*cs.pi/4, 2*2*cs.pi/4, 3*2*cs.pi/4, 0.5, 0.5, 0.5, 0.5]

    parameters = x_init+x_finish+rectangles+[1]

    solution = mng.call(parameters, initial_guess=[1.0, 0.0] * (nu*N//2))
    mng.kill()

    print(f'Solution time: {solution["solve_time_ms"]}')
    print(f'Exit status: {solution["exit_status"]}')


    # Plot solution
    # ------------------------------------
    time = np.arange(0, ts*N, ts)
    u_star = solution['solution']
    vel = u_star[0:nu*N:2]
    omega = u_star[1:nu*N:2]

    plt.subplot(311)
    plt.plot(time, vel, '-o')
    plt.ylabel('velocity')
    plt.subplot(312)
    plt.plot(time, omega, '-o')
    plt.ylabel('angular velocity')
    plt.xlabel('Time')


    # Plot trajectory
    # ------------------------------------
    nx = nz//2
    x_states = [0.0] * (nx*(N+2))
    x_states[0:nx+1] = x_init
    for t in range(0, N):
        u_t = u_star[t*nu:(t+1)*nu]

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
    plt.plot([xmin,xmax,xmax,xmin, xmin], [ymin, ymin, ymax, ymax, ymin])
    plt.plot([xmin1,xmax1,xmax1,xmin1, xmin1], [ymin1, ymin1, ymax1, ymax1, ymin1])
    plt.axis('equal')

    plt.show()

if __name__ == '__main__':
    do_build = False
    do_run = True

    if do_build:
        build()

    if do_run:
        run()