# Optimization modules
import casadi as ca
import casadi.tools
# ROS modules
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
# Standard python modules
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as anim


def generatePotentialField(obsPos):
    map_size = np.zeros((500, 500))
    i, j = map_size.shape
    g = np.zeros(map_size.shape)
    p = np.zeros(map_size.shape)
    pmax = 255
    k = 0.1# Tuning parameter
    for ls in range(i):
        for ks in range(j):
            #print('values of array: ', (ls, ks))
            g[ls, ks] = (obsPos[0][0] - obsPos[0][2]/2 - ls) + abs(obsPos[0][0] - obsPos[0][2]/2 - ls) + \
                        (ls - obsPos[0][0]-obsPos[0][2]/2 + 1) + abs(ls - obsPos[0][0]-obsPos[0][2]/2 + 1) +\
                        (obsPos[0][1] - obsPos[0][2]/2 - ks) + abs(obsPos[0][1] - obsPos[0][2]/2 - ks) + \
                        (ks - obsPos[0][1]-obsPos[0][2]/2 + 1) + abs(ks - obsPos[0][1]-obsPos[0][2]/2 + 1)
            p[ls, ks] = pmax/(1+math.exp(g[ls, ks]*k)) #(1+g[ls,ks])
    fig, ax = plt.subplots(figsize=(15, 15))
    print(p)
    ax.set_xlim([0, 100])
    ax.set_ylim([0, 100])
    #ax.matshow(p, cmap=plt.cm.Blues)
    plt.imshow(p, cmap='Blues', interpolation='nearest')
    plt.show()




def getGlobalPlan():
    # Define obstacle
    obsPos = [[25, 25, 0.2], [1, 1, 0.15]]

    # Define goal
    goal = [5, 5]
    return goal, obsPos


def main():
    goal, obsPos = getGlobalPlan()

    generatePotentialField(obsPos)





if __name__=="__main__":
    main()