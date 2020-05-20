import rospy
import numpy as np
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray, Pose
import time

x_st_0 = np.array([[0.60841257, -11.10397985,   0.84140064],
 [0.67205963, -11.02688307,   0.91994046],
 [0.72946154, -10.94503027,   0.99848027],
 [0.7802644,  -10.85892609,   1.07702009],
 [0.824155,   -10.76910139,   1.15555991],
 [0.86086274, -10.67610997,   1.23409973],
 [0.89016129, -10.58052516,   1.31263954],
 [0.91187003, -10.48293627,   1.39117936],
 [0.92585511, -10.38394497,   1.46971918],
 [0.93267961, -10.28419604,   1.53525296],
 [0.9341945, -10.18421445,   1.57603862],
 [0.93269667, -10.08422724,   1.59551225],
 [0.92993271,  -9.98426559,   1.60136675],
 [0.92692178,  -9.88431093,   1.60045351],
 [0.92411583,  -9.78435035,   1.59726565],
 [0.92162651,  -9.68438138,   1.59411858],
 [0.91940726,  -9.58440603,   1.59186261],
 [0.91736554,  -9.48442688,   1.59056726],
 [0.91541727,  -9.38444586,   1.58999328],
 [0.91350475,  -9.28446415,   1.58985211],
 [0.91159614,  -9.18448236,   1.58991508],
 [0.90967825,  -9.08450075,   1.59003769],
 [0.90774881,  -8.98451937,   1.59014621],
 [0.90581057,  -8.88453815,   1.59021373],
 [0.90386774,  -8.78455703,   1.59023791],
 [0.9019244,  -8.68457591,   1.59022392],
 [0.89998436,  -8.58459473,   1.59017205],
 [0.89805209,  -8.4846134,   1.59006833],
 [0.89613456,  -8.38463178,   1.58987732],
 [0.89424359,  -8.28464966,   1.58953712],
 [0.89239855,  -8.18466669,   1.58895829],
 [0.89062892,  -8.08468235,   1.58802894],
 [0.88897577,  -7.98469602,   1.58662812],
 [0.88749165,  -7.88470705,   1.58464812],
 [0.88623773,  -7.78471494,   1.58202361],
 [0.88527813,  -7.68471959,   1.57876119],
 [0.88467177,  -7.58472149,   1.57495877],
 [0.88446343,  -7.48472177,   1.57080078],
 [0.88467733,  -7.38472208,   1.56651382],
 [0.88531146,  -7.28571428,   1.5622693],
 [0.88531146,  -7.28571428,   1.5622693]])


print(len(x_st_0))

rospy.init_node('pose_array')
r = rospy.Rate(10)
pub = rospy.Publisher("/poseArrayTopic", PoseArray, queue_size=100)


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



def main():


    while not rospy.is_shutdown():
        poseArray = PoseArray()
        poseArray.header.stamp = rospy.Time.now()
        poseArray.header.frame_id = "/map"
        for k in range(len(x_st_0)):
            x_st = Pose()
            x_st.position.x = x_st_0[k, 0]
            x_st.position.y = x_st_0[k, 1]

            [qx, qy, qz, qw] = euler_to_quaternion(0, 0, x_st_0[k, 2])
            x_st.orientation.x = qx
            x_st.orientation.y = qy
            x_st.orientation.z = qz
            x_st.orientation.w = qw
            poseArray.poses.append(x_st)



        pub.publish(poseArray)
        r.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass