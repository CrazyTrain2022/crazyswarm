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

    # def cf1_callback(self, msg):
    #     # self.x_data.append(msg.pose.x)
    #     # self.y_data.append(msg.pose.y) 
    #     # self.z_data.append(msg.pose.z)
    #     print("Hej")


# class cfs():
#     def __init__(self, name, color, vis):
#         #super().__init__()
#         self.name = name
#         self.color = color
#         self.pos = vis.ax.scatter([], [], [], self.color, label = self.name)
#         self.x_data, self.y_data, self.z_data = [],[],[]

#     def cf_callback(self, msg):
#         self.x_data.append(msg.position.x)
#         self.y_data.append(msg.position.y) 
#         self.z_data.append(msg.position.z)
#         # print("Hej")

#     def update_plot(self, frame):
#         self.pos._offsets3d = [self.x_data, self.y_data, self.z_data]

#     def listener(self):
#         rospy.Subscriber("/mocap_qualisys/" + self.name + "/pose", PoseStamped, self.cf_callback)  #   # Topic is given by /{mocap_sys}/{subject_name}/odom



class cfs():
    def __init__(self, vis):
        
        self.cf1 = vis.ax.scatter([], [], [], 'ro', label = "Cf1")
        self.cf2 = vis.ax.scatter([], [], [], 'bo', label = "Cf2")
        self.cf1_x_data, self.cf1_y_data, self.cf1_z_data = [],[],[]
        self.cf2_x_data, self.cf2_y_data, self.cf2_z_data = [],[],[]

    def cf1_callback(self, msg):
        self.cf1_x_data.append(msg.position.x)
        self.cf1_y_data.append(msg.position.y) 
        self.cf1_z_data.append(msg.position.z)
    
    def cf2_callback(self, msg):
        self.cf2_x_data.append(msg.position.x)
        self.cf2_y_data.append(msg.position.y) 
        self.cf2_z_data.append(msg.position.z)
        # print("Hej")

    def update_plot(self, frame):
        self.cf1._offsets3d = [self.cf1_x_data, self.cf1_y_data, self.cf1_z_data]
        self.cf2._offsets3d = [self.cf2_x_data, self.cf2_y_data, self.cf2_z_data]


# def Simulate(data):
#     rospy.loginfo(data.pose.position.x)
    

if __name__ == "__main__":

    vis = Visualisation()
    # cf1 = cfs("cf1", 'bo', vis)
    # cf2 = cfs("cf2", 'ro', vis)
    cf = cfs()
    rospy.Subscriber("/mocap_qualisys/cf1/pose", PoseStamped, cf.cf_callback)
    rospy.Subscriber("/mocap_qualisys/cf2/pose", PoseStamped, cf.cf_callback)
    
    # Alternativt:

    # cf1.listener()
    # cf2.listener()
    
    ani = animation.FuncAnimation(vis.fig, cf.update_plot, init_func=vis.plot_init)
    # ani2 = animation.FuncAnimation(vis.fig, cf2.update_plot, init_func=vis.plot_init)
    plt.show()