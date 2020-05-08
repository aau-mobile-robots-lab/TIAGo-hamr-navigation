import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped
import time


def main():

    pub = rospy.Publisher('Local_Goal', PointStamped, queue_size=100)
    pub2 = rospy.Publisher('Cmd_vel', TwistStamped, queue_size=100)
    rospy.init_node('pub_goal', anonymous=True)
    rate = rospy.Rate(100)
    goal = PointStamped()
    vel = TwistStamped()

    while not rospy.is_shutdown():
        for k in range(2):
            goal.point.x = 1 + k*0.25
            goal.point.y = 2 + k*0.25
            goal.point.z = 0.4
            vel.twist.linear.x = 0.5 + k
            vel.twist.angular.z = 0.314 + k
            pub.publish(goal)
            pub2.publish(vel)
            #time.sleep(1)
            rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass