import numpy as np
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
import time
import message_filters

k  = 40


def cbMPC(goal_data, vel_data, a, b):
    goal = [goal_data.point.x + k, goal_data.point.y + k]
    print("This is the goal: ", goal)
    print(a)
    print(b)
    #print(args[0], args[1])
    #print(args[2], args[3])
    #print("This is the goal data: ", goal_data)
    #print("This is the vel data: ", vel_data)

def main():
    rospy.init_node('Goal_reader', anonymous=True)
    a = 1
    b = 2
    c = 3
    d = 4
    goal_sub = message_filters.Subscriber('Local_Goal', PointStamped)
    vel_sub = message_filters.Subscriber('Cmd_vel', TwistStamped)
    ts = message_filters.TimeSynchronizer([goal_sub, vel_sub], 10)
    ts.registerCallback(cbMPC, a, b)

    rospy.spin()

    #for k in range(10):
    #    goal = rospy.wait_for_message("Local_Goal", Point, timeout=10)
    #    x_goal = np.array([[goal.x], [goal.y], [goal.z]])
    #    print(x_goal)
    #rospy.spin()

















if __name__ == '__main__':
    main()