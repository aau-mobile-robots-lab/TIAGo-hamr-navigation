# Optimization modules
import casadi as ca
# Standard python modules
import math
import numpy as np
import numpy.matlib
# ROS specific modules
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from visualization_msgs.msg import MarkerArray, Marker
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg

# Function definitions

class CasadiMPC:

    def __init__(self, lbw, ubw, lbg, ubg):
        #publishers
        self.pub_nav_vel = rospy.Publisher('/nav_vel', Twist, queue_size=0)
        self.pub_prediction_poses = rospy.Publisher("/prediction_poses", PoseArray, queue_size=0)
        self.pub_goal = rospy.Publisher("/local_goal", PoseStamped, queue_size=0)
        #subscribers
        self.robotpose_sub = rospy.Subscriber('/robot_pose', PoseWithCovarianceStamped, self.callback_pose)
        self.goal_sub = rospy.Subscriber('/goal_pub', PoseStamped, self.callback_goal)
        self.costmap_converter_sub = rospy.Subscriber('/costmap_converter/costmap_obstacles', ObstacleArrayMsg, self.callback_so)
        self.mo_sub = rospy.Subscriber('/MO_Obstacles', ObstacleArrayMsg, self.callback_mo)
        #casadi variables
        self.lbw = np.array(lbw)
        self.ubw = np.array(ubw)
        self.lbg = np.array(lbg).T
        self.ubg = np.array(ubg).T
        self.p = np.zeros((n_states + n_states + n_MO*(N+1)*n_MOst) + n_SO*3)
        self.u0 = np.zeros((2, N))
        self.x_st_0 = np.matlib.repmat(np.array([[0], [0], [0.0]]), 1, N + 1).T
        self.goal = None
        self.MO_obs = None
        self.SO_obs = None
        self.mpc_i = 0
        self.goal_tolerance = [0.1, 0.2]

    def callback_goal(self, goal_data):  # Current way to get the goal
        yaw_goal = quaternion_to_euler(goal_data.pose.orientation.x, goal_data.pose.orientation.y, goal_data.pose.orientation.z, goal_data.pose.orientation.w)
        self.goal = np.array(([goal_data.pose.position.x], [goal_data.pose.position.y], [yaw_goal]))
        goal_data.header.stamp = rospy.Time.now()
        goal_data.header.frame_id = "/map"
        self.pub_goal.publish(goal_data)

    def callback_pose(self, pose_data):  # Update the pose either using the topics robot_pose or amcl_pose.
        yaw_pose = quaternion_to_euler(pose_data.pose.pose.orientation.x, pose_data.pose.pose.orientation.y, pose_data.pose.pose.orientation.z, pose_data.pose.pose.orientation.w)
        print('Im getting a position' )
        self.pose = np.array(([pose_data.pose.pose.position.x], [pose_data.pose.pose.position.y], [yaw_pose]))

    def callback_so(self, obs_data):
        self.SO_obs = obs_data

    def callback_mo(self, MO_data):
        self.MO_obs = []
        for k in range(len(MO_data.obstacles[:])):
            self.MO_obs.append(MO_data.obstacles[k])

    def compute_vel_cmds(self):
        if self.goal is not None and self.MO_obs is not None and self.SO_obs is not None:
            x0 = self.pose
            x_goal = self.goal
            print('This is the GOAL: ', x_goal)
            print('This is the ROBOT POSE:', x0)
            #self.dist = np.linalg.norm(x0[0] - x_goal[0], x0[1] - x_goal[1])
            #self.ori = abs(x0[2] - x_goal[2])
            #print('This is the dist:', self.dist)
            #print('This is the ori: ', self.ori)

            #if self.dist > self.goal_tolerance[0] and self.ori > self.goal_tolerance[1]:

            self.p[0:6] = np.append(x0, x_goal)

            i_pos = 6
            for k in range(N + 1):
                for i in range(n_MO):
                    self.p[i_pos + 2:i_pos + 5] = np.array([self.MO_obs[i].velocities.twist.linear.x, self.MO_obs[i].orientation.z, self.MO_obs[i].radius])

                    t_predicted = k * Ts

                    obs_x = self.MO_obs[i].polygon.points[0].x + t_predicted * self.MO_obs[i].velocities.twist.linear.x * ca.cos(self.MO_obs[i].orientation.z)
                    obs_y = self.MO_obs[i].polygon.points[0].y + t_predicted * self.MO_obs[i].velocities.twist.linear.x * ca.sin(self.MO_obs[i].orientation.z)
                    #print('This is MO x:', obs_x)
                    #print('This is MO y:', obs_y)
                    self.p[i_pos:i_pos + 2] = [obs_x, obs_y]
                    i_pos += 5

            self.p[i_pos:i_pos+n_SO*3] = closest_n_obs(self.SO_obs, x0, n_SO)
            #print('This is the static obstacles in the p vector: ', self.p[i_pos:i_pos+n_SO*3])
            #print('This is the shape of the p vector(static): ', self.p[i_pos:i_pos+n_SO*3].shape)

            x0k = np.append(self.x_st_0.reshape(3 * (N + 1), 1), self.u0.reshape(2 * N, 1))
            x0k = x0k.reshape(x0k.shape[0], 1)

            sol = solver(x0=x0k, lbx=self.lbw, ubx=self.ubw, lbg=self.lbg, ubg=self.ubg, p=self.p)

            u_sol = sol.get('x')[3 * (N + 1):].reshape((N, 2))

            self.u0 = np.append(u_sol[1:, :], u_sol[u_sol.shape[0] - 1, :], axis=0)

            self.x_st_0 = np.reshape(sol.get('x')[0:3 * (N + 1)], (N + 1, 3))
            self.x_st_0 = np.append(self.x_st_0[1:, :], self.x_st_0[-1, :].reshape((1, 3)), axis=0)
            print('This is u_sol: ', u_sol)
            cmd_vel = Twist()
            cmd_vel.linear.x = u_sol[0]
            cmd_vel.angular.z = u_sol[1]
            self.pub_nav_vel.publish(cmd_vel)

            self.mpc_i = self.mpc_i + 1

            poseArray = PoseArray()
            poseArray.header.stamp = rospy.Time.now()
            poseArray.header.frame_id = "/map"
            for k in range(len(self.x_st_0)):
                x_st = Pose()
                x_st.position.x = self.x_st_0[k, 0]
                x_st.position.y = self.x_st_0[k, 1]
                [qx, qy, qz, qw] = euler_to_quaternion(0, 0, self.x_st_0[k, 2])
                x_st.orientation.x = qx
                x_st.orientation.y = qy
                x_st.orientation.z = qz
                x_st.orientation.w = qw        #self.goal = np.array(([path_data.poses[-1].pose.position.x],[path_data.poses[1].pose.position.y],[path_data.poses[1].pose.orientation.z]))

                poseArray.poses.append(x_st)

            self.pub_prediction_poses.publish(poseArray)
            #else:
            #    print('Goal has been reached!')
        else:
            print("Goal has not been received yet. Waiting.")

def poligon2centroid(SO_data):

    if len(SO_data) == 1:
        centroid_x = SO_data[0].x
        centroid_y = SO_data[0].y
        centroid_r = 0
        return np.array([centroid_x, centroid_y, centroid_r])

    elif (len(SO_data) == 3 or len(SO_data) == 2):
        centroid_x = (SO_data[0].x+SO_data[1].x)/2
        #SO_data = SO_data[:len(SO_data) - 1]
        centroid_y = (SO_data[0].y+SO_data[1].y)/2
        start_line = np.append(SO_data[0].x, SO_data[0].y)
        end_line = np.append(SO_data[1].x, SO_data[1].y)
        centroid_r = np.linalg.norm(end_line-start_line)/2
        return np.array([centroid_x, centroid_y, centroid_r])

    else:
        SO_data = SO_data[:len(SO_data) - 1]
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
            #dist = np.linalg.norm(np.array([poly_x[k], poly_y[k]])-np.array([centroid_x, centroid_y]))
            dist = np.sqrt((poly_x[k]-centroid_x) ** 2 + (poly_y[k]-centroid_y) ** 2)
            if centroid_radius < dist:
                centroid_radius = dist
        #print('These are the radii: ', centroid_radius)
        return np.array([centroid_x, centroid_y, centroid_radius])

def closest_n_obs(SO_data, pose, n_SO):

    if len(SO_data.obstacles[:]) < n_SO:
        for i in range(len(SO_data.obstacles[:]), n_SO+1):
            fill_obs = ObstacleMsg()
            fill_obs.polygon.points = [Point32()]
            fill_obs.polygon.points[0].x = 100
            fill_obs.polygon.points[0].y = 100
            SO_data.obstacles.append(fill_obs)

    dist = np.zeros((1, len(SO_data.obstacles[:])))
    #print('After: ', len(SO_data.obstacles[:]))
    #print('These are the obs: ', SO_data.obstacles[:])
    for k in range(len(SO_data.obstacles[:])):
        print('This is for point {}'.format(k), SO_data.obstacles[k].polygon.points[:])
        #print('This is for point {}'.format(k), len(SO_data.obstacles[k].polygon.points[:]))

        [x, y, r] = poligon2centroid(SO_data.obstacles[k].polygon.points[:])
        print('This is centroid x and y: ', [x, y])
        print('This is the pose ', [pose[0], pose[1]])
        dist[0, k] = np.sqrt((pose[0]-x) ** 2 + (pose[1]-y) ** 2)

        print('This is the dist: ', dist[0, k])

    #print(np.array([[1, 2, 3]]).shape)
    n_idx = (dist).argsort()[:n_SO]

    cl_obs = np.zeros([n_SO*3])
    markerArray = MarkerArray()
    for k in range(n_SO):
        cl_obs[k*3:k*3+3] = poligon2centroid(SO_data.obstacles[n_idx[0, k]].polygon.points[:])
        print('These are the closest obs X,Y {}'.format(k+1), cl_obs[k*3:k*3+2])
        print('These are the closest obs R {}'.format(k+1), cl_obs[k*3+2:k*3+3])
        marker = Marker()
        marker.id = k
        marker.header.frame_id = '/map'
        marker.header.stamp = rospy.Time.now()
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = cl_obs[k * 3 + 2] + 0.01
        marker.scale.y = cl_obs[k * 3 + 2] + 0.01
        marker.scale.z = 0.01
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.pose.position.x = cl_obs[k * 3]
        marker.pose.position.y = cl_obs[k * 3 + 1]
        marker.pose.position.z = 0
        marker.pose.orientation.w = 1.0
        markerArray.markers.append(marker)
    pub4.publish(markerArray)
    return cl_obs

def euler_to_quaternion(roll, pitch, yaw):

        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(
            yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(
            yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(
            yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(
            yaw / 2)

        return [qx, qy, qz, qw]

def quaternion_to_euler(x, y, z, w):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1) # Not used currently
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2) # Not used currently
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return yaw

if __name__ == '__main__':
    rospy.init_node('casadi_tiago_mpc', anonymous=True)

    # MPC Parameters
    Ts = 0.1  # Timestep
    N = 40  # Horizon

    n_SO = 20

    # Robot Parameters
    safety_boundary = 0.1
    rob_diameter = 0.54
    v_max = 0.5  # m/s
    v_min = 0 # -v_max
    w_max = ca.pi/2  # rad/s
    w_min = -w_max
    acc_v_max = 0.4  # m/ss
    acc_w_max = ca.pi / 4  # rad/ss

    # Obstacle Parameters
    MO_init = np.array([[30.0, 1.0, ca.pi / 2, 0.5, 0.3],
                        [20.0, 3.5, 0.0, 0.5, 0.3],
                        [30.5, 1.5, ca.pi, 0.7, 0.2],
                        [20.0, 2.0, -ca.pi, 0.6, 0.3]])
    n_MO = len(MO_init[:, 0])
    n_MOst = 5

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
    Q[0, 0] = 5  # x
    Q[1, 1] = 5  # y
    Q[2, 2] = 0.1  # theta

    # weighing matrices (controls)
    R = np.zeros((2, 2))
    R[0, 0] = 10  # v
    R[1, 1] = 0.1  # omega

    # Weighting acc
    G = np.zeros((2, 2))
    G[0, 0] = 20  # linear acc
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
                                    rob_diameter / 2 + P[i_pos + 4] + safety_boundary)
            i_pos += 5

    k_pos = i_pos
    for k in range(N + 1):
        for i in range(n_SO):
            const_vect = ca.vertcat(const_vect, -ca.sqrt((X[0, k] - P[i_pos]) ** 2 + (X[1, k] - P[i_pos + 1]) ** 2) +
                                    (rob_diameter / 2 + P[i_pos + 2]) + safety_boundary)
            i_pos += 3
        i_pos = k_pos


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

    # Equality constraints for multiple shooting
    for k in range(n_states * (N + 1)):
        lbg += [0]
        ubg += [0]

    # Obstacles represented as inequality constraints

    for k in range((n_MO + n_SO) * (N + 1)):
        lbg += [-ca.inf]
        ubg += [0]

    # Initialize the python node = 0.5
    mpc = CasadiMPC(lbw, ubw, lbg, ubg)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        mpc.compute_vel_cmds()
        rate.sleep()