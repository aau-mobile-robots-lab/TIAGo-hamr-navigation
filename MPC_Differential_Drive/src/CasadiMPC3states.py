# Optimization modules
import casadi as ca
import casadi.tools
# Standard python modules
import time
import math as m
import numpy as np
from struct import *
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as anim


# MPC Parameters
Ts = 1/10 #0.1  # Timestep
N = 20  # Horizon

# Robot Parameters
rob_diameter = 0.54
v_max = 1  # m/s
v_min = -v_max
w_max = ca.pi/4  # rad/s
w_min = -w_max

# Obstacle Parameters
n_obstacle = 3
o_init = [[3.0, 1.0, ca.pi/2, 0.5, 0.3],
         [2.0, 3.5, 0.0, 0.5, 0.3],
         [3.5, 1.5, ca.pi, 0.7, 0.2]]

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

rhs = [v*ca.cos(theta), v*ca.sin(theta), omega]

# Obstacle states in each predictions

Ox = ca.SX.sym('Ox')
Oy = ca.SX.sym('Oy')
Oth = ca.SX.sym('Oth')
Ov = ca.SX.sym('Ov')
Or = ca.SX.sym('Or')
Ost = [Ox, Oy, Oth, Ov, Or]

n_Ost = len(Ost)

# System setup for casadi

mapping_func = ca.Function('f', [states, controls], rhs)

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

    #mapping_func_value = mapping_func(st, cont)
    #st_next_euler = st + (Ts * mapping_func_value)
    const_vect = [const_vect, st_next - st_next_RK4]

# Collision avoidance constraints

for k in range(N+1):
    for i in range(n_obstacle):
        i_pos = n_Ost * n_obstacle * k + 7 - (n_obstacle - i + 1) * n_Ost
        const_vect = [const_vect, -m.sqrt((X[1, k] - P[i_pos]) ^ 2 + (X[2, k] - P[i_pos + 1]) ^ 2) + (rob_diameter / 2
                                                                                                      + P[i_pos + 4])]

# Non-linear programming setup
    OPT_variables = [ca.reshape(X, 3 * (N + 1), 1), ca.reshape(U, 2 * N, 1)]    # Single shooting, create a vector from U [v,w,v,w,...]

nlp_prob = {
    'f': obj,
    'x': OPT_variables,
    'g': const_vect,
    'p': P
}  # Python dictionary. Works essentially like a matlab struct

solver = ca.nlpsol('solver', 'ipopt', nlp_prob)





















