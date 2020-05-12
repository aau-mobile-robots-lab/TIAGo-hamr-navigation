# Optimization modules
import casadi as ca
import casadi.tools
# ROS modules
#import rospy
#from nav_msgs.msg import Path
#from geometry_msgs.msg import Twist
#from sensor_msgs.msg import Imu
# Standard python modules
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as anim

# Function definitions

def pathcb(path_data, getplan):
    print(path_data.poses[1].pose)
    Globalpath.append(path_data.poses[1].pose.position.x)

   if Globalpath != []:
        getplan[0].unregister()

    #Simply add whatever manipulations you want to path_data


def getGlobalPlan():

    #pathmsg = rospy.wait_for_message("/move_base/GlobalPlanner/plan", Path, timeout=10 )

    rospy.Subscriber("/mobile_base_controller/cmd_vel", Twist, pathcb)
    getplan = None
    getplan = rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, pathcb, [getplan])
    rospy.spin()
    return pathmsg


def main():
    #rospy.init_node('local_planner')

    t1 = time.time()
    # State vector
    x = ca.tools.struct_symMX([ca.tools.entry("p", shape=2), "theta"])

    # Input vector
    u = ca.tools.struct_symMX(["v", "omega"])


    # State equations
    dxdt = ca.tools.struct_MX(x)

    dp_x = u["v"] * ca.cos(x["theta"])
    dp_y = u["v"] * ca.sin(x["theta"])
    dtheta = u["omega"]

    dxdt["p"] = ca.vertcat(dp_x, dp_y)
    dxdt["theta"] = dtheta

    # ODE Right-hand side
    rhs = dxdt
    f = ca.Function('f', [x, u], [rhs], ['x', 'u'], ['dx/dt'])
    # robot radius
    rr = 0.1
    # Define obstacle
    obsPos = [[0.5, 0, 0.22], [0.5, -0.5, 0.2], [0.8, -0.3, 0.3], [1.1, -0.5, 0.1]] #[0.9, -0.3, 0.2]



    # Define goal
    goal = [1.3, -0.5] #[1.3, -0.5][0.58, -0.85]



    # Runge Kutta 4th order approximation
    # dt = 1 # [s], 10 Hz sampling
    dt = ca.MX.sym("dt")

    # RK4
    k1 = f(x, u)
    k2 = f(x + dt / 2.0 * k1, u)
    k3 = f(x + dt / 2.0 * k2, u)
    k4 = f(x + dt * k3, u)
    xf = x + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)

    # Single step time propagation
    F_RK4 = ca.Function("F_RK4", [x, u, dt], [xf], ['x[k]', 'u[k]', "dt"], ['x[k+1]'])

    # Formulate Optimization problem
    opti = ca.Opti()  # casADi optimization object

    # Optimization horizon
    N = 600

    # Decision variables for states and inputs
    X = opti.variable(x.size, N+1)


    p_x = X[0, :]  # N samples of position in x from state vector X
    p_y = X[1, :]
    theta = X[2, :]

    U = opti.variable(u.size, N)  # Control inputs that are decision variables

    v = U[0, :]
    omega = U[1, :]

    # Initial state is a parameter
    x0 = opti.parameter(x.size)
    # Compute error that has to minimized
    e_x = (goal[0] - p_x)  # dist from goal in x and y
    e_y = (goal[1] - p_y)

    s = opti.variable(N)

    for obs in range(len(obsPos[:])):
        d = ca.sqrt((obsPos[obs][0] - p_x)**2 + (obsPos[obs][1] - p_y)**2)
        potential = ca.sqrt(1 / (1 + ca.exp(-100 * (d - obsPos[obs][2]))))
        opti.subject_to(d[0:-1].T >= obsPos[obs][2] + s + rr) #-1 is the first element in the opposite side of the vector

    # Gap-closing shooting constraints
    for k in range(N):
        opti.subject_to(X[:,k+1] == F_RK4(X[:,k], U[:,k], 0.05))
        print(F_RK4(X[:, k], U[:, k], 0.05))

    # Path constraints
    opti.subject_to(opti.bounded(-0.2, v, 0.2))
    opti.subject_to(opti.bounded(-3.14, omega, 3.14))

    # Initial and terminal constraints
    opti.subject_to(X[:, 0] == x0)

    opti.subject_to(X[:2, N] == goal)


    
    #print("This is the potential vector: ", potential[:])
    opti.minimize(0.1*ca.sumsqr(e_x) + 0.1*ca.sumsqr(e_y) + 0.1*ca.sumsqr(U) + 3000*ca.sumsqr(s))  #

    # Use interior point optimization
    opti.solver('ipopt', {'ipopt': {'print_level': 0}})
    # Set initial conditions
    opti.set_value(x0, [0, 0, 1.56])
    sol = opti.solve()

    x_traj = sol.value(X).T[:-1]
    u_traj = sol.value(U).T

    sol_traj = pd.DataFrame(np.hstack((x_traj, u_traj)), columns=['x', 'y', 'theta', 'v', 'omega'])

    t2 = time.time()
    print(t2-t1)

    # Plot the trajectory around the obstacle
    plt.plot(2*sol.value(e_x[0:-1])**2 + 2*sol.value(e_y[0:-1])**2+ 3000*sol.value(s)**2)#

    fig, ax = plt.subplots(figsize=(8, 8))
    c = np.zeros(len(obsPos[:]))
    for obs in range(len(obsPos[:])):
        c = plt.Circle((obsPos[obs][0], obsPos[obs][1]), radius=obsPos[obs][2], alpha=0.5)
        ax.add_patch(c)

    ax.plot(goal[0], goal[1], 'x', markersize=20)
    ax.plot(sol_traj["x"], sol_traj["y"])
    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    ax.grid()
    print(opti)

    plt.show()

    #while True:

    #pathmsg = getGlobalPlan()
    #xpos = []
    #ypos = []
    #for i in range(len(pathmsg.poses)):
    #    xpos.append(pathmsg.poses[i].pose.position.x)
    #    ypos.append(pathmsg.poses[i].pose.position.y)
    #fig, ax = plt.subplots(figsize=(8, 8))
    #ax.set_xlim([-2.5, 1])
    #ax.set_ylim([-2.5, 1])
    #ax.grid()
    #ax.plot(xpos, ypos)


    #h = 200 # This is the horizon/point along the global plan which will be used as a goal
    #goal = [pathmsg.poses[h].pose.position.x, pathmsg.poses[h].pose.position.y]
    #ax.plot(goal[0], goal[1], 'x', markersize=20)
    #ax.text(goal[0]+0.1, goal[1]+0.1, 'Goal', style='oblique', fontsize=14)
    #plt.show()

if __name__=="__main__":
    main()