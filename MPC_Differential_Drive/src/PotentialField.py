# Optimization modules
import casadi as ca
import casadi.tools
# ROS modules
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
# Standard python modules
#from mayavi import mlab

import time
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as anim
from mpl_toolkits.mplot3d import Axes3D


def generatePotentialField(obsPos):
    map_size = (500, 500)
    i, j = map_size
    p = np.zeros(map_size)
    pmax = 255
    k = 0.1  # Tuning parameter

    for obs in range(len(obsPos[:])):
        for ls in range(i):
            for ks in range(j):
                # print('values of array: ', (ls, ks))

                #g = (obsPos[obs][0] - obsPos[obs][2]/2 - ls) + abs(obsPos[obs][0] - obsPos[obs][2]/2 - ls) + \
                            #(ls - obsPos[obs][0]-obsPos[obs][2]/2 + 1) + abs(ls - obsPos[obs][0]-obsPos[obs][2]/2 + 1) +\
                            #(obsPos[obs][1] - obsPos[obs][2]/2 - ks) + abs(obsPos[obs][1] - obsPos[obs][2]/2 - ks) + \
                            #(ks - obsPos[obs][1]-obsPos[obs][2]/2 + 1) + abs(ks - obsPos[obs][1]-obsPos[obs][2]/2 + 1)
                g = abs(math.sqrt((obsPos[obs][0]-ls)**2 + (obsPos[obs][1]-ks)**2) - obsPos[obs][2])
                p[ls, ks] = pmax/(1+math.exp(g*k)) + p[ls, ks]  # (1+g[ls,ks])




    # Plot the obstacles in 3d and 2d
    fig2, ax1 = plt.subplots(figsize=(4, 4))
    #ax1.imshow(p, cmap='Blues', interpolation='nearest')
    fig1 = plt.figure()
    ax = fig1.gca(projection='3d')
    ax.set_xlim([0, 500])
    ax.set_ylim([0, 500])
    ax.set_zlim([0, 500])
    x = y = np.arange(0, 500)
    X, Y = np.meshgrid(x, y)
    ax.plot_surface(X, Y, p)
    #ax.plot_surface(X, Y, p[:, :, 1])
    ax.view_init(azim=60, elev=16)
    #fig1.show()
    #plt.show()




def getGlobalPlan():
    # Define obstacle
    obsPos = [[250, 250, 50], [100, 100, 15], [200, 200, 10], [400, 400, 35]]

    # Define goal
    goal = [5, 5]
    return goal, obsPos


def main():
    t1 = time.time()
    goal, obsPos = getGlobalPlan()

    generatePotentialField(obsPos)
    t2 = time.time()
    print(t2 - t1)



if __name__=="__main__":
    main()