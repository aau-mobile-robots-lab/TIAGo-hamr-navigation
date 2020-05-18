import rospy
import numpy as np
import casadi as ca

from costmap_converter.msg import ObstacleArrayMsg

rospy.init_node('publish_MO_obs_node', anonymous=True) #Initialize the ROS node

MO_init = np.array([[3.0, 1.0, ca.pi / 2, 0.5, 0.3],
                    [2.0, 3.5, 0.0, 0.5, 0.3],
                    [3.5, 1.5, ca.pi, 0.7, 0.2],
                    [2.0, 2.0, -ca.pi, 0.6, 0.3]]) #Define all the obstacles in the workspace.

n_MO = len(MO_init[:, 0])

pub = rospy.Publisher('/MO_Obstacles', ObstacleArrayMsg, queue_size = 100) #Setup the publisher and the topic



rate = rospy.Rate(10) # Define the publishing rate

MO_msg = ObstacleArrayMsg() # Create a msg of type ObstacleArrayMsg


for k in range(n_MO):

	MO_msg.obstacles[:]#.polygon.points.x = MO_init[0, 0]
	#MO_msg.obstacles[0].polygon.points[0].y = MO_init[k, 1]
	#MO_msg.obstacles[0].radius  = MO_init[k, 4]
	#MO_msg.obstacles[0].orientation.z = MO_init[k, 2]
	#MO_msg.obstacles[0].velocities.twist.linear.x = MO_init[k, 3]

pub.publish(MO_msg)

ros.spin()
