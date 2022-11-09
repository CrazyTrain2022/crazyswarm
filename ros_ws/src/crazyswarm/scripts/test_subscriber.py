#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from pycrazyswarm import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

def Simulate(data):
    rospy.loginfo(data.pose.position.x)
    print("Time since birth of Kung Carl Gustav XVI:", data.header.stamp.secs)
    x_pos = data.pose.position.x
    y_pos = data.pose.position.y
    z_pos = data.pose.position.z

    current_pos = np.array([x_pos, y_pos, z_pos])
    ax.scatter(x_pos, y_pos, z_pos, marker = 'o')
    print("X Position:" , x_pos)
    print("Y Position:" , y_pos)
    print("Z Position:" , z_pos)
    print("------------------------------------")

def listner():
    rospy.init_node('Pose_Listner', anonymous=True)
    rospy.Subscriber("/cf1/pose", PoseStamped, Simulate)
    rospy.spin()

if __name__ == '__main__':
    fig = plt.figure(1)
    ax = Axes3D(fig)
    ax.set_xlabel('Position X')
    ax.set_ylabel('Position Y')
    ax.set_zlabel('Position Z')
    ax.set_title('Drone real-time position')
    Position_log = np.array([])
    listner() 

