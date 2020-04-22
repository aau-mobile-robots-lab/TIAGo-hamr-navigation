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
P = ca.SX.sym('P', n_states + n_states + n_obstacle*(N+1)*n_Ost) # Parameters which include the initial state and the reference state of the robot

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
const_vect = []  # constraint vector

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
    st_next_RK4 = F_RK4(st, cont)
    const_vect = ca.vertcat(const_vect, st_next - st_next_RK4)


# Collision avoidance constraints

for k in range(N+1):
    for i in range(n_obstacle):
        i_pos = n_Ost * n_obstacle * k + 7 - (n_obstacle - i + 1) * n_Ost
        const_vect = ca.vertcat(const_vect, -ca.sqrt((X[1, k] - P[i_pos])**2 + (X[2, k] - P[i_pos + 1])**2) + (rob_diameter / 2
                                                                                                      + P[i_pos + 4]))

# Non-linear programming setup
OPT_variables = ca.vertcat(ca.reshape(X, 3 * (N + 1), 1), ca.reshape(U, 2 * N, 1))  # Single shooting, create a vector from U [v,w,v,w,...]

nlp_prob = {'f': obj,
            'x': OPT_variables,
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

Xk = ca.MX.sym('X0' + str(k + 1), 3)  # Initial conditions for the robot
w += [Xk]
lbw += [0, 0, -ca.inf]
ubw += [5, 5, ca.inf]
w0 += [0, 0, 0]

# Add constraints for each iteration

for k in range(N):
    # Constraints on the states
    Xk = ca.MX.sym('X_' + str(k+1), 3) # Make a NLP variable for the constraints on position(x,y) and the MPC
    w   += [Xk]
    lbw += [0, 0, -ca.inf]
    ubw += [5,  5, ca.inf]
    w0  += [0, 0, 0]

for k in range(N):
    # Constraints on the input
    Uk = ca.MX.sym('U_' + str(k), 2)
    w   += [Uk]
    lbw += [v_min, w_min]
    ubw += [v_max, w_max]
    w0  += [0, 0]

# Add constraints for each of the obstacles

for k in range(3*(N+1)):
    lbg += [0]
    ubg += [0]
    print(np.asarray(lbg).shape)

# Obstacles represented as inequality constraints
for n in range(n_obstacle):
    #lbg[(n+2)*(N+1)+1:(n+2)*(N+1)+(N+1)] = [-ca.inf]
    for k in range((n+3)*(N+1)+1, (n+3)*(N+1)+(N+2)):
        lbg += [-ca.inf]
        ubg += [0]

# Simulation setup

t0 = 0
x0 = np.array([[0], [0], [0.0]])
x_goal = np.array([[4], [4], [0.0]])

u0 = np.zeros((N, 2))
x_st_0 = np.matlib.repmat(x0, 1, N+1).T

t = np.array([t0])
x_ol = np.array([x0])


sim_time = 15
goal_tolerance = 0.01
mpc_max = int(sim_time/Ts) + 1



# Start MPC

mpc_i = 0
x_cl = []
u_cl = []
o_cl = np.zeros((n_obstacle, N, 5, mpc_max))
#p = np.zeros((1, 321))

t1 = time.time()

while np.linalg.norm(x0-x_goal, 2) > goal_tolerance and mpc_i < sim_time/Ts:
    mpc_time = time.time()

    p = np.append(x0, x_goal)


    for k in range(N):
        for i in range(n_obstacle):
            i_pos = n_Ost*n_obstacle*(k+1)+7-(n_obstacle-i+1)*n_Ost

            p = np.append(p, np.array([O_init[i, 2], O_init[i, 3], O_init[i, 4]]), axis=0)

            #p[0, i_pos+2:i_pos+5] = np.array([O_init[i, 2], O_init[i, 3], O_init[i, 4]])
            o_cl[i, k, 2:5, mpc_i+1] = np.array([O_init[i, 2], O_init[i, 3], O_init[i, 4]])

            # o_cl[i, k, 2:4, mpc_i+1] = np.array([O_init[i, 2], O_init[i, 3], O_init[i, 4]])

            t_predicted = (mpc_i + k)*Ts

            obs_x = O_init[i, 0] + t_predicted * O_init[i, 3] * ca.cos(O_init[i, 2])
            obs_y = O_init[i, 1] + t_predicted * O_init[i, 3] * ca.sin(O_init[i, 2])

            #p = np.append(p, [obs_x, obs_y], axis=0)
            p[0, i_pos:i_pos+2] = [obs_x, obs_y]
            o_cl[i, k, 0:2, mpc_i + 1] = [obs_x, obs_y]





    print('MPC iteration: mpc_' + str(mpc_i))
    mpc_i = mpc_i + 1


x0 = np.append(x_st_0.T.reshape(3*(N+1), 1), u0.T.reshape(2*N, 1))

# Redefine lists as ndarrays after computations

lbw = np.array(lbw).reshape(np.array(lbw).shape[0], 1)
ubw = np.array(ubw).reshape(np.array(ubw).shape[0], 1)
lbg = np.array(lbg).reshape(1, np.array(lbg).shape[0])
ubg = np.array(ubg).reshape(1, np.array(ubg).shape[0])

mpc_prob = {'x0': x0,
            'lbw': lbw,
            'ubw': ubw,
            'lbg': lbg,
            'ubg': ubg,
            'p': p
}


sol = solver('x0', mpc_prob.get('x0'), 'lbw', lbw, 'ubw', ubw, 'lbg', lbg, 'ubg', ubg, 'p', p)

t2 = time.time()
print('Total runtime is: ', t2-t1)