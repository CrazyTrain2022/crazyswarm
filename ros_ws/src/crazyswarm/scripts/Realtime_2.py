import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, TransformStamped
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import uav_trajectory
import tkinter as Tkinter
import os
from pycrazyswarm import *

import math

class CF:
    def __init__(self, ax, name, color):
        self.name = name
        self.color = color
        self.ax = ax

        self.x, self.y, self.z = None, None, None
        self.x_data, self.y_data, self.z_data = [],[],[]
        self.log, = self.ax.plot(self.x_data[0:1], self.y_data[0:1], self.z_data[0:1], c= self.color, linestyle='dotted')
        self.pos, = self.ax.plot([], [], [], self.color + 'o', label = self.name)  # Oklart om färgen funkar..
        self.traj = None
        self.distance = [] 
    
    def Show_traj(self):
        path = "."
        trajectory_files = [f for f in os.listdir(path) if os.path.isfile(os.path.join(path, f))]
        # print(trajectory_pathfiles)
        for file in trajectory_files:
            if("trajectory.csv" in file):
                if(self.name in file):
                    self.Traj_exist = True
                    self.traj = uav_trajectory.Trajectory()
                    self.traj.loadcsv(file)
                    ts = np.arange(0, self.traj.duration, 0.01)
                    self.evals = np.empty((len(ts), 15))
                    for t, k in zip(ts, range(0, len(ts))):
                        e = self.traj.eval(t)
                        self.evals[k, 0:3]  = e.pos
                    self.traj_pos = np.array([self.evals[:,0], self.evals[:,1], self.evals[:,2]]) # evals[:,2]
                    self.traj  = self.ax.plot3D(self.evals[:,0], self.evals[:,1], self.evals[:,2], color = self.color,  alpha=0.2)
        
    def update_pos(self, frame):
        self.x_data.append(self.x)
        self.y_data.append(self.y) 
        self.z_data.append(self.z)
        self.p = np.array([self.x, self.y, self.z])

        self.log.set_data(self.x_data[frame], self.y_data[frame]) # , self.cf1_z_data[:frame]
        self.log.set_3d_properties(self.z_data[frame])
        self.pos.set_data(self.x, self.y)
        self.pos.set_3d_properties(self.z)
        

        if self.traj:
            self.distance.append(min(np.linalg.norm(self.traj_pos.T-self.p, axis=1)))
            if self.distance[-1] > 1:
                print("Warning, drone error > 1 m! The distance is: ", self.distance[-1])

    def save(self):
        np.savetxt("./Log_files/" + self.name + "recordedPosition.csv", np.array([self.x_data, self.y_data, self.z_data]).T, delimiter = ',', fmt = '%10f')
          

class Visualisation:
    def __init__(self, cfs):
        # Creating figure window
        self.fig = plt.figure()
        self.ax = Axes3D(self.fig)
        self.anim_running = True
        self.save = False
        self.traj_on = False

        # initialize lists

        self.cf_list = []
        for cf in cfs:
            if cf == "cf1":
                self.cf1 = CF(self.ax, 'drone1', 'k')
                self.cf_list.append(self.cf1)
            if cf == "cf2":
                self.cf2 = CF(self.ax, 'drone2', 'b')
                self.cf_list.append(self.cf2)
            if cf == "cf3":
                self.cf3 = CF(self.ax, 'drone3', 'r')
                self.cf_list.append(self.cf3)
            if cf == "cf4":
                self.cf4 = CF(self.ax, 'drone4', 'g')
                self.cf_list.append(self.cf4)

         #, self.cf3]#, self.cf4] #, self.cf2, self.cf3, self.cf4]

    def plot_init(self):
        # Setting up plot 

        self.ax.set_xlim3d(-5, 5)
        self.ax.set_ylim3d(-5, 5)
        self.ax.set_zlim3d(0, 2.5)
        self.ax.set_xlabel('Position X [m]')
        self.ax.set_ylabel('Position Y [m]')
        self.ax.set_zlabel('Position Z [m]')
        self.ax.set_title("Drone real-time position")
        self.ax.legend()

    def Show_traj(self): # Function that plots all drone trajectories
        if self.traj_on:
            for cf in self.cf_list:
                cf.traj.pop(0).remove()   #  set_visible(False)
            self.traj_on = False

        else: 
            for cf in self.cf_list:
                cf.Show_traj()
            self.traj_on = True

    def Sim_Paus(self, ani):
        
        if self.anim_running:
            ani.event_source.stop()
            self.anim_running = False
            print("Animation paused")
        else:
            ani.event_source.start()
            self.anim_running = True
            print("Animation resumed")

    def cf1_callback(self, msg):
        if isinstance(msg.pose.position.x, float):
            self.cf1.x, self.cf1.y, self.cf1.z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
            self.cf1.p = np.array([self.cf1.x, self.cf1.y, self.cf1.z])
            self.cf1.x_data.append(self.cf1.x)
            self.cf1.y_data.append(self.cf1.y) 
            self.cf1.z_data.append(self.cf1.z)

    def cf2_callback(self, msg):
        if isinstance(msg.pose.position.x, float):
            self.cf2.x, self.cf2.y, self.cf2.z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
            self.cf2.p = np.array([self.cf2.x, self.cf2.y, self.cf2.z])
            self.cf2.x_data.append(self.cf2.x)
            self.cf2.y_data.append(self.cf2.y) 
            self.cf2.z_data.append(self.cf2.z)

    def cf3_callback(self, msg):
        if isinstance(msg.pose.position.x, float):
            self.cf3.x, self.cf3.y, self.cf3.z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
            self.cf3.p = np.array([self.cf3.x, self.cf3.y, self.cf3.z])
            self.cf3.x_data.append(self.cf3.x)
            self.cf3.y_data.append(self.cf3.y) 
            self.cf3.z_data.append(self.cf3.z)

    def cf4_callback(self, msg):
        if isinstance(msg.pose.position.x, float):
            self.cf4.x, self.cf4.y, self.cf4.z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
            self.cf4.p = np.array([self.cf4.x, self.cf4.y, self.cf4.z])
            self.cf4.x_data.append(self.cf4.x)
            self.cf4.y_data.append(self.cf4.y) 
            self.cf4.z_data.append(self.cf4.z)
    

    def update_plot(self, frame):
        # allcfs = Crazyswarm().allcfs
        # for cf in allcfs.crazyflies:
        #     print(cf.prefix())
        for cf in self.cf_list:
            cf.update_pos(frame)

    def save_log(self, name):
        self.fig.savefig("./Log_files/" + str(name) + ".png")
        for cf in self.cf_list:
            cf.save()
        self.plot_error()
        
    def plot_error(self):
        self.fig2, self.ax2 = plt.subplots()
        
        for cf in self.cf_list:
            t = np.linspace(0,len(cf.distance), num = len(cf.distance))
            self.ax2.plot(t,cf.distance, color = cf.color, label = cf.name)

        self.ax2.set_title("Distance to trajectory")
        self.ax2.set_xlabel("Time")
        self.ax2.set_ylabel("Error from reference")
        self.ax2.legend()
        self.ax2.grid()
        self.fig2.savefig("./Log_files/Path_error.png")
        plt.show()

if __name__ == "__main__":
    
    allcfs = Crazyswarm().allcfs
    i = 1

    cfs = []
    for cf in allcfs.crazyflies:
        cfs.append = cf.prefix
    
    vis = Visualisation(cfs)

    rospy.init_node('Pose_Listener')
    for cf in cfs:
        if cf == "cf1":
            cf1 = rospy.Subscriber("/qualisys/cf1/pose", PoseStamped, vis.cf1_callback)
        if cf == "cf2":
            cf2 = rospy.Subscriber("/qualisys/cf2/pose", PoseStamped, vis.cf2_callback)
        if cf == "cf3":
            cf1 = rospy.Subscriber("/qualisys/cf3/pose", PoseStamped, vis.cf3_callback)
        if cf == "cf4":
            cf2 = rospy.Subscriber("/qualisys/cf4/pose", PoseStamped, vis.cf4_callback)
        
    ani = animation.FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init) # oklart om true eller ej!
    
    mainwindow = Tkinter.Tk()
    mainwindow.title("CrazyTrain - Realtime simulation control")
    mainwindow.geometry("400x300")
    frame = Tkinter.Frame(mainwindow)  #inside box
    frame.pack()
    start_print = Tkinter.Button(frame, text = "Start/Pause visualisation",  bg='green', command = lambda : vis.Sim_Paus(ani))
    start_print.pack()
    traj_print = Tkinter.Button(frame, text="Show trajectory", bg='grey', command = lambda : vis.Show_traj())
    traj_print.pack()

    entry1 = Tkinter.Entry(mainwindow)
    entry1.insert(0, "Name of saved file")
    entry1.pack()
    save_print = Tkinter.Button(frame, text = "Save plot to file",  bg='grey', command = lambda : vis.save_log(entry1.get())) 
    save_print.pack()
    plot = Tkinter.Button(frame, text = "Plot error",  bg='grey', command = lambda : vis.plot_error()) 
    plot.pack()
    plt.show()