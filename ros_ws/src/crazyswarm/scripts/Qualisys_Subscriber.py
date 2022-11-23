import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, TransformStamped
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import uav_trajectory
from pycrazyswarm import *
import tkinter as Tkinter
import os



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

    def Show_traj(self):
        traj_lst = []
        i = 0
        path = "."
        trajectory_files = [f for f in os.listdir(path) if os.path.isfile(os.path.join(path, f))]
        # print(trajectory_pathfiles)
        for file in trajectory_files:
            if("trajectory.csv" in file):
                traj_lst.append(uav_trajectory.Trajectory())
                traj_lst[i].loadcsv(file)
                ts = np.arange(0, traj_lst[i].duration, 0.01)
                evals = np.empty((len(ts), 15))
                for t, k in zip(ts, range(0, len(ts))):
                    e = traj_lst[i].eval(t)
                    evals[k, 0:3]  = e.pos

                trajectory = self.ax.plot3D(evals[:,0], evals[:,1], evals[:,2])
                i += 1
        
def Sim_Paus():
    global anim_running
    if anim_running:
        ani.event_source.stop()
        anim_running = False
        print("Animation paused")
    else:
        ani.event_source.start()
        anim_running = True
        print("Animation resumed")

class cfs:
    def __init__(self, vis):
        
        self.cf1 = vis.ax.scatter([], [], [], 'ro', label = "Cf1")
        self.cf2 = vis.ax.scatter([], [], [], 'bo', label = "Cf2")
        self.cf3 = vis.ax.scatter([], [], [], 'ko', label = "Cf3")
        self.cf4 = vis.ax.scatter([], [], [], 'go', label = "Cf4")

        self.cf1_x_data, self.cf1_y_data, self.cf1_z_data = [],[],[]
        self.cf2_x_data, self.cf2_y_data, self.cf2_z_data = [],[],[]
        self.cf3_x_data, self.cf3_y_data, self.cf3_z_data = [],[],[]
        self.cf4_x_data, self.cf4_y_data, self.cf4_z_data = [],[],[]

    def cf1_callback(self, msg):
        self.cf1_x_data.append(msg.pose.position.x)
        self.cf1_y_data.append(msg.pose.position.y) 
        self.cf1_z_data.append(msg.pose.position.z)
    
    def cf2_callback(self, msg):
        self.cf2_x_data.append(msg.pose.position.x)
        self.cf2_y_data.append(msg.pose.position.y) 
        self.cf2_z_data.append(msg.pose.position.z)
    
    def cf3_callback(self, msg):
        self.cf3_x_data.append(msg.pose.position.x)
        self.cf3_y_data.append(msg.pose.position.y) 
        self.cf3_z_data.append(msg.pose.position.z)
    
    def cf4_callback(self, msg):
        self.cf4_x_data.append(msg.pose.position.x)
        self.cf4_y_data.append(msg.pose.position.y) 
        self.cf4_z_data.append(msg.pose.position.z)

    def update_plot(self, frame):
        self.cf1._offsets3d = [self.cf1_x_data, self.cf1_y_data, self.cf1_z_data]
        self.cf2._offsets3d = [self.cf2_x_data, self.cf2_y_data, self.cf2_z_data]
        self.cf3._offsets3d = [self.cf3_x_data, self.cf3_y_data, self.cf3_z_data]
        self.cf4._offsets3d = [self.cf4_x_data, self.cf4_y_data, self.cf4_z_data]


if __name__ == "__main__":
    mainwindow = Tkinter.Tk()
    mainwindow.title("CrazyTrain - Realtime simulation control")
    mainwindow.geometry("350x300")

    frame = Tkinter.Frame(mainwindow)  #inside box
    frame.pack()
    start_print = Tkinter.Button(frame, text = "Start/Pause visualisation",  bg='green', command = Sim_Paus)
    start_print.pack()
    # start_print = Tkinter.Button(frame, text = "Save position to file",  bg='grey', command = save_csv) # variable = bool_save, onvalue=0, offvalue=1, command=save_csv)
    # start_print.pack()

    vis = Visualisation()
    vis.Show_traj()
    cf = cfs(vis)
    rospy.init_node('Pose_Listener')
    cf1 = rospy.Subscriber("/qualisys/cf1/pose", PoseStamped, cf.cf1_callback)
    cf2 = rospy.Subscriber("/qualisys/cf2/pose", PoseStamped, cf.cf2_callback)
    cf3 = rospy.Subscriber("/qualisys/cf3/pose", PoseStamped, cf.cf3_callback)
    cf4 = rospy.Subscriber("/qualisys/cf4/pose", PoseStamped, cf.cf4_callback)

    ani = animation.FuncAnimation(vis.fig, cf.update_plot, init_func=vis.plot_init)
    plt.show()