import rospy
import numpy as np
import casadi as ca
import time
from geometry_msgs.msg import Point32
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from visualization_msgs.msg import MarkerArray, Marker

rospy.init_node('publish_MO_obs_node', anonymous=True)  # Initialize the ROS node

MO_init = np.array([[-3.85, -0.22, ca.pi, -0.6, 0.3],
                    [-4, 1, 0.0, 0.6, 0.3],
                    [3.91, -11.63, -ca.pi, 0.6, 0.2],
                    [-4.41, -9.5, ca.pi, -0.6, 0.3]])  # Define all the obstacles in the workspace.

n_MO = len(MO_init[:, 0])

pub = rospy.Publisher('/MO_Obstacles', ObstacleArrayMsg, queue_size=1)  # Setup the publisher and the topic
pub1 = rospy.Publisher('/visualization_marker', MarkerArray, queue_size=1)
rate = rospy.Rate(10)  # Define the publishing rate
step = 0.1

for Ts in np.arange(0.1, 5, step):
    MO_msg = ObstacleArrayMsg()  # Create a msg of type ObstacleArrayMsg
    MO_msg.header.stamp = rospy.Time.now()
    MO_msg.header.frame_id = '/map'
    markerArray = MarkerArray()

    for k in range(n_MO):
        # Calculate the next step using runge kutta 4. order
        k1 = np.array((MO_init[k, 3] * ca.cos(MO_init[k, 2]), MO_init[k, 3] * ca.sin(MO_init[k, 2])))
        k2 = np.array((MO_init[k, 3] * ca.cos(MO_init[k, 2]) + step / 2 * k1[0],
                       MO_init[k, 3] * ca.sin(MO_init[k, 2]) + step / 2 * k1[1]))
        k3 = np.array((MO_init[k, 3] * ca.cos(MO_init[k, 2]) + step / 2 * k2[0],
                       MO_init[k, 3] * ca.sin(MO_init[k, 2]) + step / 2 * k2[1]))
        k4 = np.array((MO_init[k, 3] * ca.cos(MO_init[k, 2]) + step * k3[0],
                       MO_init[k, 3] * ca.sin(MO_init[k, 2]) + step * k3[0]))

        xf = np.array((MO_init[k, 0], MO_init[k, 1]) + step / 6 * (k1 + 2 * k2 + 2 * k3 + k4))
        MO_init[k, 0] = xf[0]
        MO_init[k, 1] = xf[1]
        print('This is for n_MO {}'.format(k), xf)
        # Add obstacle parameters and states at time Ts to the obstacle message
        MO_msg.obstacles.append(ObstacleMsg())
        MO_msg.obstacles[k].id = k
        MO_msg.obstacles[k].polygon.points = [Point32()]
        MO_msg.obstacles[k].polygon.points[0].x = xf[0]
        MO_msg.obstacles[k].polygon.points[0].y = xf[1]
        MO_msg.obstacles[k].radius = MO_init[k, 4]
        MO_msg.obstacles[k].orientation.z = MO_init[k, 2]
        MO_msg.obstacles[k].velocities.twist.linear.x = MO_init[k, 3]
    #print(MO_msg.obstacles[0])
    rate.sleep()
    pub.publish(MO_msg)

    print('It should be published now')
