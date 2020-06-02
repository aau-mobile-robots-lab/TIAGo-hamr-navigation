# Optimization modules
import casadi as ca
import casadi.tools
# Standard python modules
import time
import math as m
import numpy as np
from struct import *
import pandas as pd
import numpy.matlib
import matplotlib.pyplot as plt
import rospy

import matplotlib.animation as anim
# ROS specific modules and msgs


import message_filters

# Function definitions

def shift(Ts, t0, x0, u_sol, F_RK4):

    st = x0
    cont = np.array([u_sol[0], u_sol[1]]) #u_sol[0, :].T
    st_next = F_RK4(st, cont)
    x0 = st_next
    #mapping_func_value = mapping_func(st, cont)
    #x0 = st + (Ts*mapping_func_value)
    t0 = t0 + Ts
    u0 = np.append(u_sol[1:, :], u_sol[u_sol.shape[0]-1, :], axis=0)

    return t0, x0, u0


def plt_fnc(state, predict, goal, t, u_cl, SO_init, MO_init):

    plt.figure(1)
    plt.grid()
    plt.text(goal[0] - 0.15, goal[1] - 0.2, 'Goal', style='oblique', fontsize=10)
    plt.text(-0.1, 0.15, 'Start', style='oblique', fontsize=10)
    plt.plot(state.T[:, 0], state.T[:, 1])
    plt.plot(0, 0, 'bo')
    #plt.plot(predict[:, 0], predict[:, 1])
    plt.plot(goal[0], goal[1], 'ro')
    plt.xlabel('X-position [Meters]')
    plt.ylabel('Y-position [Meters]')
    plt.title('MPC in python')
    for obs in range(len(SO_init[:])):
        c1 = plt.Circle((SO_init[obs][0], SO_init[obs][1]), radius=SO_init[obs][2], alpha=0.5)
        plt.gcf().gca().add_artist(c1)
    for obs in range(len(MO_init[:])):
        plt.quiver(MO_init[obs][0], MO_init[obs][1], 1*ca.cos(MO_init[obs][2]), 1*ca.sin(MO_init[obs][2]))
        c2 = plt.Circle((MO_init[obs][0], MO_init[obs][1]), radius=MO_init[obs][4], alpha=0.5, color='r')
        plt.gcf().gca().add_artist(c2)
    #plt.xlim(-10, 10)
    #plt.ylim(-10, 10)

    fig, (ax1, ax2) = plt.subplots(2)
    fig.suptitle('Control Signals From MPC Solution')
    ax1.plot(t, u_cl[:, 0])
    ax1.grid()
    ax2.plot(t, u_cl[:, 1])
    ax2.grid()
    ax2.set_ylabel('Angular Velocity [m/s]')
    ax2.set_xlabel('Time [s]')
    ax1.set_ylabel('Linear Velocity [rad/s]')
    ax1.set_xlabel('Time [s]')

    plt.show()
    return state, predict, goal, t

# MPC Parameters
Ts = 0.1 # Timestep
N = 40  # Horizon

# Robot Parameters
rob_diameter = 0.54
v_max = 1  # m/s
v_min = -v_max
w_max = ca.pi/4  # rad/s
w_min = -w_max
acc_v_max = 0.4    # m/ss
acc_w_max = ca.pi/4   # rad/ss

# Obstacle Parameters
MO_init = np.array([[3.0, 1.0, ca.pi/2, 0.5, 0.3],
         [2.0, 3.5, 0.0, 0.5, 0.3],
         [3.5, 1.5, ca.pi, 0.7, 0.2],
         [2.0, 2.0, -ca.pi,   0.6, 0.3]])
n_MO = len(MO_init[:, 0])

SO_init = np.array([[1.0, 3.0, 0.3],
           [9.0, 1.5, 0.1],
           [2.0, 2.0, 0.3],
           [6.0, 2.5, 0.2]])
n_SO = len(SO_init[:, 0])

# System Model
x = ca.SX.sym('x')
y = ca.SX.sym('y')
theta = ca.SX.sym('theta')
states = ca.vertcat(x, y, theta)
n_states = 3  # len([states])

# Control system
v = ca.SX.sym('v')
omega = ca.SX.sym('omega')
controls = ca.vertcat(v, omega)
n_controls = 2  # len([controls])

rhs = ca.vertcat(v*ca.cos(theta), v*ca.sin(theta), omega)

# Obstacle states in each predictions
MOx = ca.SX.sym('MOx')
MOy = ca.SX.sym('MOy')
MOth = ca.SX.sym('MOth')
MOv = ca.SX.sym('MOv')
MOr = ca.SX.sym('MOr')
Ost = [MOx, MOy, MOth, MOv, MOr]

n_MOst = len(Ost)

# System setup for casadi
mapping_func = ca.Function('f', [states, controls], [rhs])

# Declare empty system matrices
U = ca.SX.sym('U', n_controls, N)


# Parameters:initial state(x0), reference state (xref), obstacles (O)
P = ca.SX.sym('P', n_states + n_states + n_MO*(N+1)*n_MOst)  # Parameters which include the initial state and the reference state of the robot

X = ca.SX.sym('X', n_states, (N+1))  # Prediction matrix

# Objective Function and Constraints

# weighing matrices (states)
Q = np.zeros((3, 3))
Q[0, 0] = 1  # x
Q[1, 1] = 5  # y
Q[2, 2] = 0.1  # theta

# weighing matrices (controls)
R = np.zeros((2, 2))
R[0, 0] = 0.5  # v
R[1, 1] = 0.05  # omega

# Weighting acc
G = np.zeros((2, 2))
G[0, 0] = 50  # linear acc
G[1, 1] = 5   # Angular acc

obj = 0  # Objective Q and R
const_vect = np.array([])  # constraint vector

# Lift
st = X[:, 0]
const_vect = ca.vertcat(const_vect, st-P[0:3])


#M = 4  # Fixed step size per interval
#for j in range(M):
k1 = mapping_func(states, controls)
k2 = mapping_func(states + Ts / 2 * k1, controls)
k3 = mapping_func(states + Ts / 2 * k2, controls)
k4 = mapping_func(states + Ts * k3, controls)
xf = states + Ts / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

# Single step time propagation
F_RK4 = ca.Function("F_RK4", [states, controls], [xf], ['x[k]', 'u[k]'], ['x[k+1]'])


# Calculate the objective function and constraints
for k in range(N):
    st = X[:, k]
    cont = U[:, k]
    if k < N-1:
        cont_next = U[:, k+1]

    obj = obj + ca.mtimes(ca.mtimes((st - P[3:6]).T, Q), (st - P[3:6])) + \
          ca.mtimes(ca.mtimes(cont.T, R), cont) + \
          ca.mtimes(ca.mtimes((cont-cont_next).T, G), (cont-cont_next))

    st_next = X[:, k+1]
    #mapping_func_value = mapping_func(st, cont)
    #st_next_euler = st + (Ts*mapping_func_value)
    st_next_RK4 = F_RK4(st, cont)
    const_vect = ca.vertcat(const_vect, st_next - st_next_RK4)

# Collision avoidance constraints

for k in range(N+1):
    for i in range(n_MO):
        i_pos = n_MOst * n_MO * (k+1) + 7 - (n_MO - (i+1) + 1) * n_MOst
        const_vect = ca.vertcat(const_vect, -ca.sqrt((X[0, k] - P[i_pos-1])**2 + (X[1, k] - P[i_pos])**2) +
                                (rob_diameter / 2 + P[i_pos + 3]))

for k in range(N+1):
    for i in range(n_SO):
        const_vect = ca.vertcat(const_vect, -ca.sqrt((X[0, k] - SO_init[i, 0])**2 + (X[1, k] - SO_init[i, 1])**2) +
                   (rob_diameter / 2 + SO_init[i, 2]))


# Non-linear programming setup
OPT_variables = ca.vertcat(ca.reshape(X, 3 * (N + 1), 1), ca.reshape(U, 2 * N, 1))  #  Single shooting, create a vector from U [v,w,v,w,...]

nlp_prob = {'x': OPT_variables,
            'f': obj,
            'g': const_vect,
            'p': P
}  # Python dictionary. Works essentially like a matlab struct

solver = ca.nlpsol('solver', 'ipopt', nlp_prob)

# Start with an empty NLP
lbw = []
ubw = []
J = 0
g = []
lbg = []
ubg = []
lbw += [0, 0, -ca.inf]
ubw += [5, 5, ca.inf]

# Add constraints for each iteration
for k in range(N):
    # Constraints on the states
    lbw += [0, 0, -ca.inf]
    ubw += [5,  5, ca.inf]

for k in range(N):
    # Constraints on the input
    lbw += [v_min, w_min]
    ubw += [v_max, w_max]

# Add constraints for each of the obstacles
for k in range(n_states*(N+1)):
    lbg += [0]
    ubg += [0]

# Obstacles represented as inequality constraints

for k in range((n_MO + n_SO)*(N+1)):
    lbg += [-ca.inf]
    ubg += [0]

# Simulation setup
t0 = np.array([0])
x0 = np.array([[0], [0], [0.0]])

#x_goal = np.array([[4], [4], [0.0]])

u0 = np.zeros((2, N))
x_st_0 = np.matlib.repmat(x0, 1, N+1).T

t = t0
x_ol = x0
sim_time = 15

goal_tolerance = 0.01
mpc_max = int(sim_time/Ts) + 1

# Start MPC
mpc_i = 0
x_cl = np.zeros((21, 3))
u_cl = np.zeros((1, 2))
o_cl = np.zeros((n_MO, N+1, 5, mpc_max))
p = np.zeros((n_states + n_states + n_MO*(N+1)*n_MOst))

print("I got to this place")
# Setup ROS communication
rospy.init_node('Python_MPC', anonymous=True)
print("i to got this place!")
pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=100)
rate = rospy.Rate(10)
print("i got to this place")
goal_sub = message_filters.Subscriber('/Local_Goal', PointStamped)
#vel_sub = message_filters.Subscriber('Cmd_vel', TwistStamped)
pose_sub = message_filters.Subscriber('/amcl_pose', PoseWithCovarianceStamped)
ts = message_filters.TimeSynchronizer([goal_sub, pose_sub], 100)


initt = False
def cbmpc(goal_data, pose_data, lbw, ubw, lbg, ubg, t0, pub, N):
    # t1 = time.time()
    # np.linalg.norm(x0-x_goal, 2) > goal_tolerance and
    # while mpc_i < sim_time/Ts:
    global initt
    if initt is False:
        u0 = np.zeros((2, N))
        t0 = np.array([0])
        initt = True
    # Read message from the global planner
    x_goal = np.array([[goal_data.point.x], [goal_data.point.y], [goal_data.point.z]])
    # Get pose from the AMCL
    x0 = np.array([pose_data.pose.pose.position.x], [pose_data.pose.pose.position.y], [pose_data.pose.pose.orientation.z])
    #x0 = np.array([[pose_data.pose.position.x], [pose_data.pose.position.y],[pose_data.pose.orientation.z]])
    # Define prediction horizon based upon pose of robot
    x_st_0 = np.matlib.repmat(x0, 1, N + 1).T

    # Create msg for sending velocity commands to the base.
    cmd_vel = Twist()

    # Append current initial position and goal position
    p[0:6] = np.append(x0, x_goal)
    print("This is 6 p entries :", p[0:6])

    for k in range(N+1):
        for i in range(n_MO):
            i_pos = n_MOst*n_MO*(k+1)+6-(n_MO-i)*n_MOst

            p[i_pos+2:i_pos+5] = np.array([MO_init[i, 2], MO_init[i, 3], MO_init[i, 4]])
            #p[i_pos+2:i_pos+5] = np.array([MO_init[i, 2], MO_init[i, 3], MO_init[i, 4]])

            #o_cl[i, k, 2:5, mpc_i+1] = np.array([MO_init[i, 2], MO_init[i, 3], MO_init[i, 4]])

            t_predicted = k*Ts

            obs_x = MO_init[i, 0] + t_predicted * MO_init[i, 3] * ca.cos(MO_init[i, 2])
            obs_y = MO_init[i, 1] + t_predicted * MO_init[i, 3] * ca.sin(MO_init[i, 2])

            p[i_pos:i_pos+2] = [obs_x, obs_y]
            #o_cl[i, k, 0:2, mpc_i + 1] = [obs_x, obs_y]

        x0k = np.append(x_st_0.reshape(3*(N+1), 1), u0.reshape(2*N, 1))
        x0k = x0k.reshape(x0k.shape[0], 1)

        # Redefine lists as ndarrays after computations
        lbw = np.array(lbw)
        ubw = np.array(ubw)
        lbg = np.array(lbg).T
        ubg = np.array(ubg).T

        sol = solver(x0=x0k, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg, p=p)

        u_sol = sol.get('x')[3*(N+1):].reshape((N, 2))
        #x_cl = np.append(x_cl, np.reshape(sol.get('x')[0:3*(N+1)], (N+1, 3)), axis=0)
        #u_cl = np.append(u_cl, np.array([[u_sol[0], u_sol[1]]]), axis=0)
        #x_ol = np.append(x_ol, x0, axis=1)
        #t = np.append(t, t0, axis=0)

        cmd_vel.linear.x = u_sol[0]
        cmd_vel.angular.z = u_sol[1]
        pub.publish(cmd_vel)

        [t0, x0, u0] = shift(Ts, t0, x0, u_sol, F_RK4)

        x_st_0 = np.reshape(sol.get('x')[0:3*(N+1)], (N+1, 3))

        x_st_0 = np.append(x_st_0[1:, :], x_st_0[-1, :].reshape((1, 3)), axis=0)

        #print('MPC iteration: mpc_' + str(mpc_i))
        #mpc_i = mpc_i + 1
        time.sleep(0.5)
        print("This is x0 after manipulation: ", x0)

ts.registerCallback(cbmpc, lbw, ubw, lbg, ubg, t0, pub, N)
rospy.spin()
#t2 = time.time()
#print('Total runtime is: ', t2-t1)

#plt_fnc(x_ol, x_cl, x_goal, t, u_cl, SO_init, MO_init)

