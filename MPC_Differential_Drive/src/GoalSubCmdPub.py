import numpy as np
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped, Point32
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
import message_filters
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg



def main():

    rospy.init_node('Goal_reader', anonymous=True)

    class Listener:
        def __init__(self):
            self.goal = None
            self.pose = None
            self.k = 0
            self.n_SO = 10

        def goal_cb(self, goal_data):
            self.goal = np.array((goal_data.point.x, goal_data.point.y))
            self.print_msg()

        def pose_cb(self, pose_data):
            self.pose = pose_data
            self.print_msg()

        def obs_cb(self, SO_data):
            self.SO_obs = SO_data
            print('Size of the SO_obs: ', len(self.SO_obs.obstacles[:]))

            if len(self.SO_obs.obstacles[:]) < self.n_SO:
                for i in range(len(SO_data.obstacles[:]), self.n_SO + 1):
                    fill_obs = ObstacleMsg()
                    fill_obs.polygon.points = [Point32()]
                    fill_obs.polygon.points[0].x = 1000
                    fill_obs.polygon.points[0].y = 1000
                    self.SO_obs.obstacles.append(fill_obs)
            print('This is the size of the SO_obs after additional obs: ', self.SO_obs.obstacles)

        def print_msg(self):
            self.k = self.k + 1
            #print("This is the goal ", self.goal)
            #print("This is the pose ", self.pose)
            #print("Counter ", self.k)



    listen = Listener()

    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, listen.pose_cb)
    rospy.Subscriber('/Local_Goal', PointStamped, listen.goal_cb)
    rospy.Subscriber('/costmap_converter/costmap_obstacles', ObstacleArrayMsg, listen.obs_cb)

    rospy.spin()























    #goal_sub = message_filters.Subscriber('Local_Goal', PointStamped)
    #vel_sub = message_filters.Subscriber('Cmd_vel', TwistStamped)
    #pose_sub = message_filters.Subscriber('/amcl_pose', PoseWithCovarianceStamped)
    #pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, posecb)
    #ts = message_filters.TimeSynchronizer([pose_sub], 10) # goal_sub, vel_sub,
    #ts.registerCallback(cbMPC)
    #rospy.spin()

    #for k in range(10):
    #    goal = rospy.wait_for_message("Local_Goal", Point, timeout=10)
    #    x_goal = np.array([[goal.x], [goal.y], [goal.z]])
    #    print(x_goal)
    #rospy.spin()








#k = False
#def cbMPC(pose_data):
    #goal = [goal_data.point.x , goal_data.point.y]
    #print("This is the goal: ", goal)
#    print("This is the pose", [pose_data.pose.pose.position.x, pose_data.pose.pose.position.y, pose_data.pose.pose.orientation.z])
    #print(args[0], args[1])
    #print(args[2], args[3])
    #print("This is the goal data: ", goal_data)
    #print("This is the vel data: ", vel_data)



#def posecb(pose_data):
#    print("This is the pose", [pose_data.pose.pose.position.x, pose_data.pose.pose.position.y, pose_data.pose.pose.orientation.z])








if __name__ == '__main__':
    main()