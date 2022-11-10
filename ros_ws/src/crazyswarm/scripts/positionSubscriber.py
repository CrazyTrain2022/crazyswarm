
import numpy as np
import warnings

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped

import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection

from matplotlib.animation import FuncAnimation


#var ska denna raden stå? 
dtype=object

class Visualiser:
    def __init__(self):

        self.fig, self.axe = plt.subplots()
        self.axe = plt.axes(projection="3d")
        self.ln = self.axe.scatter([], [], [],'ro')

        self.x_data, self.y_data, self.z_data = [],[],[]

    def plot_init(self):
        self.axe.set_xlim3d(-5, 5)
        self.axe.set_ylim3d(-5, 5)
        self.axe.set_zlim3d(0, 5)
        return self.ln
     

    def pos_callback(self, msg):

        x = msg.pose.position.x
        y= msg.pose.position.y
        z = msg.pose.position.z

        self.x_data.append(x)
        self.y_data.append(y) 
        self.z_data.append(z)

    def update_plot(self, frame):

        self.ln._offsets3d = [self.x_data, self.y_data, self.z_data]
        
        self.export_positions = np.array([self.x_data, self.y_data, self.z_data]).T
        np.savetxt("./ros_ws/src/crazyswarm/scripts/recordedPosition.csv", self.export_positions, delimiter=",",fmt = '%10.3f')
   

        return self.ln


rospy.init_node('posPlot')
vis = Visualiser()
sub = rospy.Subscriber('/cf1/pose', PoseStamped, vis.pos_callback)
ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
plt.show(block=True) 

