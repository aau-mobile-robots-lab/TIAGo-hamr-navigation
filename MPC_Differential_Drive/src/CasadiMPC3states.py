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
import matplotlib.animation as anim

# Function definitions


def shift(Ts, t0, x0, u_sol, F_RK4):

    st = x0
    cont = u_sol[0, :].T
    st_next = F_RK4(st, cont)
    x0 = st_next
    #mapping_func_value = mapping_func(st, cont)
    #x0 = st + (Ts*mapping_func_value)


    t0 = t0 + Ts
    u0 = np.append(u_sol[1:, :], u_sol[u_sol.shape[0]-1, :], axis=0)

    return t0, x0, u0

def plt_fnc(state, predict, goal, t):
    plt.plot(state.T[:, 0], state.T[:, 1])
    #plt.plot(predict[:, 0], predict[:, 1])
    plt.plot(goal[0], goal[1], 'ro')
    plt.show()


# MPC Parameters
Ts = 0.1  # Timestep
N = 20  # Horizon

# Robot Parameters
rob_diameter = 0.54
v_max = 1  # m/s
v_min = -v_max
w_max = ca.pi/4  # rad/s
w_min = -w_max

# Obstacle Parameters
n_obstacle = 3
O_init = np.array([[3.0, 1.0, ca.pi/2, 0.5, 0.3],
         [2.0, 3.5, 0.0, 0.5, 0.3],
         [3.5, 1.5, ca.pi, 0.7, 0.2]])

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
Ox = ca.SX.sym('Ox')
Oy = ca.SX.sym('Oy')
Oth = ca.SX.sym('Oth')
Ov = ca.SX.sym('Ov')
Or = ca.SX.sym('Or')
Ost = [Ox, Oy, Oth, Ov, Or]

n_Ost = len(Ost)

# System setup for casadi
mapping_func = ca.Function('f', [states, controls], [rhs])

# Declare empty system matrices
U = ca.SX.sym('U', n_controls, N)


# Parameters:initial state(x0), reference state (xref), obstacles (O)
P = ca.SX.sym('P', n_states + n_states + n_obstacle*(N+1)*n_Ost)  # Parameters which include the initial state and the reference state of the robot

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


obj = 0  # Objective Q and R
const_vect = np.array([])  # constraint vector

# Lift
st = X[:, 0]
const_vect = ca.vertcat(const_vect, st-P[0:3])


# Runge-kutta 4th order integration
# RK4
X0 = ca.MX.sym('X0', 3)

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
    obj = obj + ca.mtimes(ca.mtimes((st - P[3:6]).T, Q), (st - P[3:6])) + ca.mtimes(ca.mtimes(cont.T, R), cont)
    st_next = X[:, k+1]
    mapping_func_value = mapping_func(st, cont)
    st_next_euler = st + (Ts*mapping_func_value)
    #st_next_RK4 = F_RK4(st, cont)
    const_vect = ca.vertcat(const_vect, st_next - st_next_euler)



# Collision avoidance constraints

for k in range(N+1):
    for i in range(n_obstacle):
        i_pos = n_Ost * n_obstacle * (k+1) + 7 - (n_obstacle - (i+1) + 1) * n_Ost
        const_vect = ca.vertcat(const_vect, -ca.sqrt((X[0, k] - P[i_pos-1])**2 + (X[1, k] - P[i_pos])**2) +
                                (rob_diameter / 2 + P[i_pos + 3]))



# Non-linear programming setup
OPT_variables = ca.vertcat(ca.reshape(X, 3 * (N + 1), 1), ca.reshape(U, 2 * N, 1))  # Single shooting, create a vector from U [v,w,v,w,...]

nlp_prob = {'x': OPT_variables,
            'f': obj,
            'g': const_vect,
            'p': P
}  # Python dictionary. Works essentially like a matlab struct

solver = ca.nlpsol('solver', 'ipopt', nlp_prob)

# Start with an empty NLP
w = []
w0 = []
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
for k in range(n_obstacle*(N+1)):
    lbg += [0]
    ubg += [0]

# Obstacles represented as inequality constraints
for n in range(n_obstacle):
    # lbg[(n+2)*(N+1)+1:(n+2)*(N+1)+(N+1)] = [-ca.inf]
    for k in range((n+3)*(N+1)+1, (n+3)*(N+1)+(N+2)):
        lbg += [-ca.inf]
        ubg += [0]

# Simulation setup
t0 = np.array([0])
x0 = np.array([[0], [0], [0.0]])
x_goal = np.array([[4], [4], [0.0]])

u0 = np.zeros((2, N))
x_st_0 = np.matlib.repmat(x0, 1, N+1).T

t = t0
x_ol = x0
sim_time = 15
goal_tolerance = 0.01
mpc_max = int(sim_time/Ts) + 1



# Start MPC
mpc_i = 0
x_cl = np.zeros((mpc_max+2))
u_cl = np.zeros((1, 2))
o_cl = np.zeros((n_obstacle, N+1, 5, mpc_max))
p = np.zeros((n_states + n_states + n_obstacle*(N+1)*n_Ost))

t1 = time.time()

while np.linalg.norm(x0-x_goal, 2) > goal_tolerance and mpc_i < sim_time/Ts:
    mpc_time = time.time()

    p[0:6] = np.append(x0, x_goal)

    for k in range(N+1):
        for i in range(n_obstacle):
            i_pos = n_Ost*n_obstacle*(k+1)+6-(n_obstacle-i)*n_Ost

            p[i_pos+2:i_pos+5] = np.array([O_init[i, 2], O_init[i, 3], O_init[i, 4]])
            p[i_pos+2:i_pos+5] = np.array([O_init[i, 2], O_init[i, 3], O_init[i, 4]])

            o_cl[i, k, 2:5, mpc_i+1] = np.array([O_init[i, 2], O_init[i, 3], O_init[i, 4]])

            t_predicted = (mpc_i + k)*Ts

            obs_x = O_init[i, 0] + t_predicted * O_init[i, 3] * ca.cos(O_init[i, 2])
            obs_y = O_init[i, 1] + t_predicted * O_init[i, 3] * ca.sin(O_init[i, 2])

            p[i_pos:i_pos+2] = [obs_x, obs_y]
            o_cl[i, k, 0:2, mpc_i + 1] = [obs_x, obs_y]

    x0k = np.append(x_st_0.T.reshape(3*(N+1), 1), u0.reshape(2*N, 1))
    x0k = x0k.reshape(x0k.shape[0], 1)

    # Redefine lists as ndarrays after computations

    lbw = np.array(lbw)
    ubw = np.array(ubw)
    lbg = np.array(lbg).T
    ubg = np.array(ubg).T

    sol = solver(x0=x0k, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg, p=p)

    u_sol = sol.get('x')[3*(N+1):].reshape((N, 2))
    x_cl = sol.get('x')[0:3*(N+1)].reshape((N+1, 3))
    u_cl = np.append(u_cl, u_sol[0, :], axis=0)
    x_ol = np.append(x_ol, x0, axis=1)
    t = np.append(t, t0, axis=0)

    [t0, x0, u0] = shift(Ts, t0, x0, u_sol, F_RK4)

    x_st_0 = sol.get('x')[0:3*(N+1)].reshape((N+1, 3))

    x_st_0 = np.append(x_st_0[1:, :], x_st_0[-1, :])


    print('MPC iteration: mpc_' + str(mpc_i))
    mpc_i = mpc_i + 1

t2 = time.time()
print('Total runtime is: ', t2-t1)

plt_fnc(x_ol, x_cl, x_goal, t)

