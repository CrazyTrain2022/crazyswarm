import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, TransformStamped
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import uav_trajectory
from pycrazyswarm import *

class Visualisation:
    def __init__(self):
        self.fig = plt.figure()
        self.ax = Axes3D(self.fig)
        # self.x_data, self.y_data, self.z_data = [],[],[]
        # self.cf1 = self.ax.scatter([], [], [],'ro')
        # self.cf2 = self.ax.scatter([], [], [],'bo')
    
    def plot_init(self):
        # Setting up plot 
        self.ax.set_xlim3d(-5, 5)
        self.ax.set_ylim3d(-5, 5)
        self.ax.set_zlim3d(0, 3)
        self.ax.set_xlabel('Position X [m]')
        self.ax.set_ylabel('Position Y [m]')
        self.ax.set_zlabel('Position Z [m]')
        self.ax.set_title('Drone real-time position')
        # return self.ln

    def cf1_callback(self, msg):
        # self.x_data.append(msg.pose.x)
        # self.y_data.append(msg.pose.y) 
        # self.z_data.append(msg.pose.z)
        print("Hej")


class cf():
    def __init__(self, name, color, vis):
        #super().__init__()
        self.name = name
        self.color = color
        self.pos = vis.ax.scatter([], [], [], self.color, label = self.name)
        self.x_data, self.y_data, self.z_data = [],[],[]

    def cf_callback(self, msg):
        # self.x_data.append(msg.x)
        # self.y_data.append(msg.y) 
        # self.z_data.append(msg.z)
        print("Hej")

    def listener(self):
        sub = rospy.Subscriber("/mocap_qualisys/" + self.name + "/pose", PoseStamped, self.cf_callback)  #   # Topic is given by /{mocap_sys}/{subject_name}/odom


# def Simulate(data):
#     rospy.loginfo(data.pose.position.x)
    

if __name__ == "__main__":

    
    vis = Visualisation()

    cf1 = cf("cf1", "bo", vis)
    ani = animation.FuncAnimation(vis.fig, vis.cf1_callback, init_func=vis.plot_init)
    plt.show()