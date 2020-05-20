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
# ROS specific modules
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from costmap_converter.msg import ObstacleArrayMsg

# Function definitions
def shift(u_sol):
    u0 = np.append(u_sol[1:, :], u_sol[u_sol.shape[0] - 1, :], axis=0)
    return u0


def plt_fnc(state, predict, goal, t, u_cl, SO_init, MO_init):

    plt.figure(1)
    plt.grid()
    plt.text(goal[0] - 0.15, goal[1] - 0.2, 'Goal', style='oblique', fontsize=10)
    plt.text(-0.1, 0.15, 'Start', style='oblique', fontsize=10)
    plt.plot(state.T[:, 0], state.T[:, 1])
    plt.plot(0, 0, 'bo')
    # plt.plot(predict[:, 0], predict[:, 1])
    plt.plot(goal[0], goal[1], 'ro')
    plt.xlabel('X-position [Meters]')
    plt.ylabel('Y-position [Meters]')
    plt.title('MPC in python')
    for obs in range(len(SO_init[:])):
        c1 = plt.Circle((SO_init[obs][0], SO_init[obs][1]), radius=SO_init[obs][2], alpha=0.5)
        plt.gcf().gca().add_artist(c1)
    for obs in range(len(MO_init[:])):
        plt.quiver(MO_init[obs][0], MO_init[obs][1], 1 * ca.cos(MO_init[obs][2]), 1 * ca.sin(MO_init[obs][2]))
        c2 = plt.Circle((MO_init[obs][0], MO_init[obs][1]), radius=MO_init[obs][4], alpha=0.5, color='r')
        plt.gcf().gca().add_artist(c2)
    # plt.xlim(-10, 10)
    # plt.ylim(-10, 10)

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

def poligon2centroid(SO_data):
    SO_data = SO_data[:len(SO_data)-1]



    if len(SO_data) < 2:
        centroid_x = SO_data[0].x
        centroid_y = SO_data[0].y
        centroid_r = 0
        return np.array([centroid_x, centroid_y, centroid_r])
    elif len(SO_data) == 2:
        centroid_x = (SO_data[0].x+SO_data[1].x)/2
        centroid_y = (SO_data[0].y+SO_data[1].y)/2
        start_line = np.append(SO_data[0].x, SO_data[0].y)
        end_line = np.append(SO_data[1].x, SO_data[1].y)
        centroid_r = np.linalg.norm(end_line-start_line)/2
        return np.array([centroid_x, centroid_y, centroid_r])

    else:
        poly_x = []
        poly_y = []
        for k in range(len(SO_data)):
            poly_x.append(SO_data[k].x)
            poly_y.append(SO_data[k].y)

        x_mean = np.mean(poly_x)
        y_mean = np.mean(poly_y)

        x = poly_x - x_mean
        y = poly_y - y_mean

        #create shifted matrix for counter clockwise bounderies
        xp = np.append(x[1:], x[0])
        yp = np.append(y[1:], y[0])

        #calculate the twice signed area of the elementary triangle formed by
        #(xi,yi) and (xi+1,yi+1) and the origin.
        a = np.dot(x, yp) - np.dot(xp, y)

        #Sum of the half of these areas
        area = np.sum(a)/2

        if area < 0:
            area = -area

        #calculate centroid of the shifted
        xc = np.sum(np.dot((x+xp), a))/(6*area)
        yc = np.sum(np.dot((y+yp), a))/(6*area)

        #shift back to original place
        centroid_x = xc + x_mean
        centroid_y = yc + y_mean
        centroid_radius = 0

        #calculate radius
        for k in range(len(SO_data)):
            dist = np.linalg.norm(np.array([poly_x[k], poly_y[k]])-np.array([centroid_x, centroid_y]))
            if centroid_radius < dist:
                centroid_radius = dist
        return np.array([centroid_x, centroid_y, centroid_radius])

def closest_n_obs(SO_data, pose, n_SO):
    dist = np.zeros((1, len(SO_data.obstacles[:])))
    for k in range(len(SO_data.obstacles[:])):
        [x, y, r] = poligon2centroid(SO_data.obstacles[k].polygon.points[:])

        dist[0, k] = np.linalg.norm(pose[0]-x, pose[1]-y)

    print(dist)
    print(np.array([[1, 2, 3]]).shape)
    n_idx = (-dist).argsort()[:n_SO]

    cl_obs = np.zeros([1, n_SO*3])

    for k in range(n_SO):
        cl_obs[k*3:k*3+3] = poligon2centroid(SO_data.obstacles[n_idx[0, k]].polygon.points[:])
    print('These are the closest obstacles: ', cl_obs)
    return cl_obs


# MPC Parameters
Ts = 0.1  # Timestep
N = 40  # Horizon

# Robot Parameters
rob_diameter = 0.54
v_max = 1  # m/s
v_min = -v_max
w_max = ca.pi / 4  # rad/s
w_min = -w_max
acc_v_max = 0.4  # m/ss
acc_w_max = ca.pi / 4  # rad/ss

# Obstacle Parameters
MO_init = np.array([[3.0, 1.0, ca.pi / 2, 0.5, 0.3],
                    [2.0, 3.5, 0.0, 0.5, 0.3],
                    [3.5, 1.5, ca.pi, 0.7, 0.2],
                    [2.0, 2.0, -ca.pi, 0.6, 0.3]])
n_MO = len(MO_init[:, 0])

SO_init = np.array([[3.0, 2.5],
                    [6.0, 3.0],
                    [7.0, 5.5],
                    [4.0, 6.0]])
#n_SO = len(SO_init[:, 0])
n_SO = 1

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

rhs = ca.vertcat(v * ca.cos(theta), v * ca.sin(theta), omega)

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
P = ca.SX.sym('P', n_states + n_states + n_MO * (N + 1) * n_MOst + n_SO*3)

X = ca.SX.sym('X', n_states, (N + 1))  # Prediction matrix

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
G[1, 1] = 5  # Angular acc

obj = 0  # Objective Q and R
const_vect = np.array([])  # constraint vector

# Lift
st = X[:, 0]
const_vect = ca.vertcat(const_vect, st - P[0:3])

# M = 4  # Fixed step size per interval
# for j in range(M):
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
    if k < N - 1:
        cont_next = U[:, k + 1]

    obj = obj + ca.mtimes(ca.mtimes((st - P[3:6]).T, Q), (st - P[3:6])) + \
          ca.mtimes(ca.mtimes(cont.T, R), cont) + \
          ca.mtimes(ca.mtimes((cont - cont_next).T, G), (cont - cont_next))

    st_next = X[:, k + 1]
    # mapping_func_value = mapping_func(st, cont)
    # st_next_euler = st + (Ts*mapping_func_value)
    st_next_RK4 = F_RK4(st, cont)
    const_vect = ca.vertcat(const_vect, st_next - st_next_RK4)

# Collision avoidance constraints
i_pos = 6
for k in range(N + 1):
    for i in range(n_MO):
        #i_pos = n_MOst * n_MO * (k + 1) + 7 - (n_MO - (i + 1) + 1) * n_MOst
        const_vect = ca.vertcat(const_vect, -ca.sqrt((X[0, k] - P[i_pos]) ** 2 + (X[1, k] - P[i_pos + 1]) ** 2) +
                                (rob_diameter / 2 + P[i_pos + 4]))
        i_pos += 5

for k in range(N + 1):
    for i in range(n_SO):
        const_vect = ca.vertcat(const_vect, -ca.sqrt((X[0, k] - P[i_pos]) ** 2 + (X[1, k] - P[i_pos + 1]) ** 2) +
                                (rob_diameter / 2 + P[i_pos + 2]))


# Non-linear programming setup
OPT_variables = ca.vertcat(ca.reshape(X, 3 * (N + 1), 1),
                           ca.reshape(U, 2 * N, 1))  # Single shooting, create a vector from U [v,w,v,w,...]

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
lbw += [-ca.inf, -ca.inf, -ca.inf]
ubw += [ca.inf, ca.inf, ca.inf]

# Add constraints for each iteration
for k in range(N):
    # Constraints on the states
    lbw += [-ca.inf, -ca.inf, -ca.inf]
    ubw += [ca.inf, ca.inf, ca.inf]

for k in range(N):
    # Constraints on the input
    lbw += [v_min, w_min]
    ubw += [v_max, w_max]

# Add constraints for each of the obstacles
for k in range(n_states * (N + 1)):
    lbg += [0]
    ubg += [0]

# Obstacles represented as inequality constraints

for k in range((n_MO + n_SO) * (N + 1)):
    lbg += [-ca.inf]
    ubg += [0]

# Initialize the python node
rospy.init_node('Python_MPC', anonymous=True)

class CasadiMPC:

    def __init__(self, lbw, ubw, lbg, ubg):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub1 = rospy.Publisher("/poseArrayTopic", PoseArray, queue_size=100)
        self.lbw = np.array(lbw)
        self.ubw = np.array(ubw)
        self.lbg = np.array(lbg).T
        self.ubg = np.array(ubg).T
        self.p = np.zeros((n_states + n_states + n_MO * (N + 1) * n_MOst)+n_SO*3)
        self.u0 = np.zeros((2, N))
        self.x_st_0 = np.matlib.repmat(np.array([[0], [0], [0.0]]), 1, N + 1).T
        self.goal = None
        self.MO_obs = None
        self.SO_obs = None
        self.mpc_i = 0

    def goal_cb(self, goal_data):  # Current way to get the goal
        #self.goal = np.array(([path_data.poses[-1].pose.position.x],[path_data.poses[1].pose.position.y],[path_data.poses[1].pose.orientation.z]))
        self.goal = np.array(([goal_data.pose.position.x], [goal_data.pose.position.y], [goal_data.pose.orientation.z]))
        #print('This is the goal: ', self.goal)

    def pose_cb(self, pose_data):  # Update the pose either using the topics robot_pose or amcl_pose.
        self.pose = np.array(([pose_data.pose.pose.position.x], [pose_data.pose.pose.position.y], [pose_data.pose.pose.orientation.z]))
        #print('This is the pose: ', self.pose)
        self.compute_vel_cmds()

    def obs_cb(self, obs_data):
        self.SO_obs = obs_data
        #for k in range(len(obs_data.obstacles[:])):
        #    #obs_temp = [[obs_data.obstacles[k].polygon.points[0].x], [obs_data.obstacles[k].polygon.points[0].y]]
        #    self.SO_obs.append(obs_data.obstacles[k])


    def MO_obs_cb(self, MO_data):
        self.MO_obs = []
        for k in range(len(MO_data.obstacles[:])):
            self.MO_obs.append(MO_data.obstacles[k])

    def compute_vel_cmds(self):
        if self.goal is not None and self.MO_obs is not None and self.SO_obs is not None:

            x0 = self.pose
            x_goal = self.goal
            self.p[0:6] = np.append(x0, x_goal)

            for k in range(N + 1):
                for i in range(n_MO):
                    i_pos = n_MOst * n_MO * (k + 1) + 6 - (n_MO - i) * n_MOst

                    self.p[i_pos + 2:i_pos + 5] = np.array([self.MO_obs[i].velocities.twist.linear.x, self.MO_obs[i].orientation.z, self.MO_obs[i].radius])

                    t_predicted = k * Ts

                    obs_x = self.MO_obs[i].polygon.points[0].x + t_predicted * self.MO_obs[i].velocities.twist.linear.x * ca.cos(self.MO_obs[i].orientation.z)
                    obs_y = self.MO_obs[i].polygon.points[0].y + t_predicted * self.MO_obs[i].velocities.twist.linear.x * ca.sin(self.MO_obs[i].orientation.z)

                    self.p[i_pos:i_pos + 2] = [obs_x, obs_y]
            i_pos += 5

            self.p[i_pos:i_pos+n_SO*3] = closest_n_obs(self.SO_obs, x0, n_SO)


            x0k = np.append(self.x_st_0.reshape(3 * (N + 1), 1), self.u0.reshape(2 * N, 1))
            x0k = x0k.reshape(x0k.shape[0], 1)

            sol = solver(x0=x0k, lbx=self.lbw, ubx=self.ubw, lbg=self.lbg, ubg=self.ubg, p=self.p)

            u_sol = sol.get('x')[3 * (N + 1):].reshape((N, 2))

            self.u0 = shift(u_sol)

            self.x_st_0 = np.reshape(sol.get('x')[0:3 * (N + 1)], (N + 1, 3))
            self.x_st_0 = np.append(self.x_st_0[1:, :], self.x_st_0[-1, :].reshape((1, 3)), axis=0)
            print(self.x_st_0)
            time.sleep(25)
            cmd_vel = Twist()
            cmd_vel.linear.x = u_sol[0]
            cmd_vel.angular.z = u_sol[1]
            self.pub.publish(cmd_vel)

            self.mpc_i = self.mpc_i + 1

            poseArray = PoseArray()
            poseArray.header.stamp = rospy.Time.now()
            poseArray.header.frame_id = "/map"
            for k in range(len(self.x_st_0)):
                x_st = Pose()
                x_st.position.x = self.x_st_0[k, 0]
                x_st.position.y = self.x_st_0[k, 1]
                x_st.orientation.z = self.x_st_0[k, 2]
                poseArray.poses.append(x_st)

            self.pub1.publish(poseArray)
        else:
            print("Goal has not been received yet. Waiting.")

mpc = CasadiMPC(lbw, ubw, lbg, ubg)
#rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, mpc.pose_cb)
rospy.Subscriber('/robot_pose', PoseWithCovarianceStamped, mpc.pose_cb)
rospy.Subscriber('/goal_pub', PoseStamped, mpc.goal_cb)
rospy.Subscriber('/costmap_converter/costmap_obstacles', ObstacleArrayMsg, mpc.obs_cb)
rospy.Subscriber('/MO_Obstacles', ObstacleArrayMsg, mpc.MO_obs_cb)


rospy.spin()


#plt_fnc(x_ol, x_cl, x_goal, t, u_cl, SO_init, MO_init)

