import casadi as ca
import casadi.tools

import numpy as np
import pandas as pd

import matplotlib.pyplot as plt


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
N = 50

# Decision variables for states and inputs
X = opti.variable(x.size, N+1)

p_x = X[0,:]  # N samples of position in x from state vector X
p_y = X[1,:]
theta = X[2,:]

U = opti.variable(u.size, N)  # Control inputs that are decision variables

v = U[0,:]
omega = U[1,:]

# Initial state is a parameter
x0 = opti.parameter(x.size)

# Gap-closing shooting constraints
for k in range(N):
   opti.subject_to(X[:,k+1] == F_RK4(X[:,k], U[:,k], 0.1))

# Path constraints
opti.subject_to(opti.bounded(-0.2, v, 0.2))
opti.subject_to(opti.bounded(-2.5, omega, 2.5))

# Initial and terminal constraints
opti.subject_to(X[:,0] == x0)


# Compute error that has to minimized
e_x = (0.8 - p_x)  # dist from goal in x and y
e_y = (0.4 - p_y)


s = opti.variable(N)

d = ca.sqrt((0.5 - p_x)**2 + (0.15 - p_y)**2)

# potential = ca.sqrt(1 / (1 + ca.exp(5 * (d - 0.2))))

opti.subject_to(d[0:-1].T >= s + 0.2)  # -1 is the first element in the opposite side of the vector

opti.minimize(3*ca.sumsqr(e_x) + 3*ca.sumsqr(e_y) + 0.01*ca.sumsqr(U) + 1000*ca.sumsqr(s))


# Use interior point optimization
opti.solver('ipopt', {'ipopt': {'print_level': 0}})


opti.set_value(x0, [0, 0, 0])
sol = opti.solve()

x_traj = sol.value(X).T[:-1]
u_traj = sol.value(U).T

sol_traj = pd.DataFrame(np.hstack((x_traj, u_traj)), columns=['x', 'y', 'theta', 'v', 'omega'])

# Plot the trajectory around the obstacle
plt.plot(2*sol.value(e_x[0:-1])**2 + 2*sol.value(e_y[0:-1])**2 + 3000*sol.value(s)**2)

fig, ax = plt.subplots(figsize=(8, 8))

c = plt.Circle((0.5, 0.15), radius=0.2, alpha=0.5)

ax.plot(0.8, 0.4, 'x', markersize=20)
ax.plot(sol_traj["x"], sol_traj["y"])
ax.add_patch(c)

ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])

ax.grid()
plt.show()
