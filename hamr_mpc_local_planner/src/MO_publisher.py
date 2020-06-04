import rospy
import numpy as np
import casadi as ca
import sys
from datetime import datetime
from geometry_msgs.msg import Point32, Point, PointStamped
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from visualization_msgs.msg import MarkerArray, Marker

def main():
    class MO_publisher:
        def __init__(self):
            rospy.init_node('publish_MO_obs_node', anonymous=True)  # Initialize the ROS node
                                                                        # MO_init structure: [x, y, th, v, r]
            self.MO_init = np.array([[-3.85, -0.22, -1.57, 0.5, 0.3],  # x is the width of the room 0 is approx in the middle of the room.
                                     [-4, 1, -ca.pi, -0.5, 0.3],         # y is 0 all the way in the right of the room the more negative the further left
                                     [3.91, -11.63, ca.pi, 0.5, 0.2],   # ca.pi moves up the screen decreasing this moves it to the right with change to left at -ca.pi
                                     [-4.41, -9.5, ca.pi, 0.5, 0.3]])   # Define all the obstacles in the workspace.

            self.n_MO = len(self.MO_init[:, 0])
            self.pub = rospy.Publisher('/MO_Obstacles', ObstacleArrayMsg, queue_size=1)  # Setup the publisher and the topic
            self.pub_mo_viz = rospy.Publisher('/moving_obs', MarkerArray, queue_size=1)
            self.rate = rospy.Rate(10)  # Define the publishing rate
            self.step = 0.1
            self.MOMarkerArray = MarkerArray()
            self.counter = 0

        def selection_proc(self):
            selection = raw_input("Press q to pick new points\nPress l to load a saved set of points.\nPress another key to continue with the current points\nKeypress: ")
            if selection is "Q" or selection is "q":
                for i in range(self.n_MO):
                    Pt_msg = rospy.wait_for_message("/clicked_point", PointStamped, timeout=None)
                    self.MO_init[i, 0] = Pt_msg.point.x
                    self.MO_init[i, 1] = Pt_msg.point.y

                np.save('../../../../MO_obs/MO_matrix{}'.format(datetime.now().strftime("%d%H:%M")), self.MO_init)
                print('Saved at time: {}'.format(datetime.now().strftime("%d%H:%M")))
                mo_pub.publisher()

            if selection is "L" or selection is "l":
                file_date = raw_input("Please write the day and time of the file that you want to access.\ndate: ")
                self.MO_init = np.load('../../../../MO_obs/MO_matrix{}.npy'.format(file_date))
                mo_pub.publisher()
            else:
                mo_pub.publisher()

        def publisher(self):

            while not rospy.is_shutdown():
                MO_msg = ObstacleArrayMsg()  # Create a msg of type ObstacleArrayMsg
                MO_msg.header.stamp = rospy.Time.now()
                MO_msg.header.frame_id = '/map'


                for k in range(self.n_MO):
                    # Calculate the next step using runge kutta 4. order
                    k1 = np.array((self.MO_init[k, 3] * ca.cos(self.MO_init[k, 2]), self.MO_init[k, 3] * ca.sin(self.MO_init[k, 2])))
                    k2 = np.array((self.MO_init[k, 3] * ca.cos(self.MO_init[k, 2]) + self.step / 2 * k1[0],
                                   self.MO_init[k, 3] * ca.sin(self.MO_init[k, 2]) + self.step / 2 * k1[1]))
                    k3 = np.array((self.MO_init[k, 3] * ca.cos(self.MO_init[k, 2]) + self.step / 2 * k2[0],
                                   self.MO_init[k, 3] * ca.sin(self.MO_init[k, 2]) + self.step / 2 * k2[1]))
                    k4 = np.array((self.MO_init[k, 3] * ca.cos(self.MO_init[k, 2]) + self.step * k3[0],
                                   self.MO_init[k, 3] * ca.sin(self.MO_init[k, 2]) + self.step * k3[0]))

                    xf = np.array((self.MO_init[k, 0], self.MO_init[k, 1]) + self.step / 6 * (k1 + 2 * k2 + 2 * k3 + k4))
                    self.MO_init[k, 0] = xf[0]
                    self.MO_init[k, 1] = xf[1]
                    print('This is for n_MO {}'.format(k), xf)
                    # Add obstacle parameters and states at time Ts to the obstacle message
                    MO_msg.obstacles.append(ObstacleMsg())
                    MO_msg.obstacles[k].id = k
                    MO_msg.obstacles[k].polygon.points = [Point32()]
                    MO_msg.obstacles[k].polygon.points[0].x = xf[0]
                    MO_msg.obstacles[k].polygon.points[0].y = xf[1]
                    MO_msg.obstacles[k].radius = self.MO_init[k, 4]
                    MO_msg.obstacles[k].orientation.z = self.MO_init[k, 2]
                    MO_msg.obstacles[k].velocities.twist.linear.x = self.MO_init[k, 3]
                self.counter += 1
                if self.counter is 25:
                    for k in range(self.n_MO):
                        self.MO_init[k, 3] = -self.MO_init[k, 3]
                        self.counter = 0
                mo_pub.rviz_publisher(MO_msg)
                self.rate.sleep()
                self.pub.publish(MO_msg)

                print('This is the {}. iteration of current direction'.format(self.counter))

        def rviz_publisher(self, MO_msg):

            for i in range(self.n_MO):
                marker = Marker()
                marker.id = i
                marker.header.frame_id = 'map'
                marker.header.stamp = rospy.Time.now()
                marker.type = marker.CYLINDER
                marker.action = marker.ADD
                marker.scale.x = 2 * MO_msg.obstacles[i].radius
                marker.scale.y = 2 * MO_msg.obstacles[i].radius
                marker.scale.z = 0.3
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker.pose.position.x = MO_msg.obstacles[i].polygon.points[0].x
                marker.pose.position.y = MO_msg.obstacles[i].polygon.points[0].y
                marker.pose.position.z = 0
                marker.pose.orientation.w = 1.0
                self.MOMarkerArray.markers.append(marker)
            self.pub_mo_viz.publish(self.MOMarkerArray)
    mo_pub = MO_publisher()
    mo_pub.selection_proc()

if __name__ == "__main__":
    main()