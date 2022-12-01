import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, TransformStamped
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import uav_trajectory
import tkinter as Tkinter
import os
import csv
import sys

sys.path.append("/home/crazycrowd/CrazyTrain/CrazyTrain2022/GUI/")
import gui_main_frame

class Visualisation:
    def __init__(self):
        # Creating figure window
        self.fig = plt.figure()
        self.ax = Axes3D(self.fig)
        self.anim_running = True
        self.save = False
        self.traj_on = False

        self.cf1_x_data, self.cf1_y_data, self.cf1_z_data = [], [], []
        self.cf2_x_data, self.cf2_y_data, self.cf2_z_data = [], [], []
        self.cf3_x_data, self.cf3_y_data, self.cf3_z_data = [], [], []
        self.cf4_x_data, self.cf4_y_data, self.cf4_z_data = [], [], []

        self.cf1_log, = self.ax.plot(self.cf1_x_data[0:1], self.cf1_y_data[0:1], self.cf1_z_data[0:1], c='black', linestyle='dotted') # self.ax.plot3D([], [], [], c='black', linestyle='dotted') #    # 
        self.cf2_log, = self.ax.plot(self.cf2_x_data[0:1], self.cf2_y_data[0:1], self.cf2_z_data[0:1], c='blue', linestyle='dotted')
        self.cf3_log, = self.ax.plot(self.cf3_x_data[0:1], self.cf3_y_data[0:1], self.cf3_z_data[0:1], c='red', linestyle='dotted')
        self.cf4_log, = self.ax.plot(self.cf4_x_data[0:1], self.cf4_y_data[0:1], self.cf4_z_data[0:1], c='green', linestyle='dotted')
        
        self.cf1_pos, = self.ax.plot([], [], [], 'ko', label = "Cf1")
        self.cf2_pos, = self.ax.plot([], [], [], 'bo', label = "Cf2")
        self.cf3_pos, = self.ax.plot([], [], [], 'ro', label = "Cf3")
        self.cf4_pos, = self.ax.plot([], [], [], 'go', label = "Cf4")

        self.distance1 = []
        self.distance2 = []
        self.distance3 = []
        self.distance4 = []
        self.traj_lst = []
        self.traj1_pos = np.empty([1,2],dtype=float)
        self.traj2_pos = np.empty([1,2],dtype=float)
        self.traj3_pos = np.empty([1,2],dtype=float)
        self.traj4_pos = np.empty([1,2],dtype=float)
        self.emergency_active = False

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
    def emergency(self):
        os.system('gnome-terminal -- bash ../../../../../GUI/bash_scripts/emergency.sh')
    def Show_traj(self): # Function that plots all drone trajectories
        if self.traj_on:
            for traj in self.traj_lst:
                traj.set_visible(False)
                #traj.pop(0).remove() eventuellt
            self.traj_on = False
        else:
            i = 0
            path = "."
            trajectory_files = [f for f in os.listdir(path) if os.path.isfile(os.path.join(path, f))]
            # print(trajectory_pathfiles)
            for file in trajectory_files:
                if("trajectory.csv" in file):
                    if("drone1" in file):
                        self.traj1 = uav_trajectory.Trajectory()
                        self.traj1.loadcsv(file)
                        ts = np.arange(0, self.traj1.duration, 0.01)
                        evals = np.empty((len(ts), 15))
                        for t, k in zip(ts, range(0, len(ts))):
                            e = self.traj1.eval(t)
                            evals[k, 0:3]  = e.pos
                        self.traj1_pos = np.array([evals[:,0], evals[:,1]])
                        trajectory, = self.ax.plot3D(evals[:,0], evals[:,1], evals[:,2], color = 'k',  alpha=0.2)
                        trajectory.set_visible(True)
                        self.traj_lst.append(trajectory)

                    if("drone2" in file):
                        self.traj2 = uav_trajectory.Trajectory()
                        self.traj2.loadcsv(file)
                        ts = np.arange(0, self.traj2.duration, 0.01)
                        evals = np.empty((len(ts), 15))
                        for t, k in zip(ts, range(0, len(ts))):
                            e = self.traj2.eval(t)
                            evals[k, 0:3]  = e.pos
                        self.traj2_pos = np.array([evals[:,0], evals[:,1]]) 
                        trajectory, = self.ax.plot3D(evals[:,0], evals[:,1], evals[:,2], color = 'b', alpha=0.2)
                        trajectory.set_visible(True)
                        self.traj_lst.append(trajectory)

                    if("drone3" in file):
                        self.traj3 = uav_trajectory.Trajectory()
                        self.traj3.loadcsv(file)
                        ts = np.arange(0, self.traj3.duration, 0.01)
                        evals = np.empty((len(ts), 15))
                        for t, k in zip(ts, range(0, len(ts))):
                            e = self.traj3.eval(t)
                            evals[k, 0:3]  = e.pos
                        self.traj3_pos = np.array([evals[:,0], evals[:,1]])
                        trajectory, = self.ax.plot3D(evals[:,0], evals[:,1], evals[:,2], color = 'r', alpha=0.2)
                        trajectory.set_visible(True)
                        self.traj_lst.append(trajectory)

                    if("drone4" in file):
                        self.traj4 = uav_trajectory.Trajectory()
                        self.traj4.loadcsv(file)
                        ts = np.arange(0, self.traj4.duration, 0.01)
                        evals = np.empty((len(ts), 15))
                        for t, k in zip(ts, range(0, len(ts))):
                            e = self.traj4.eval(t)
                            evals[k, 0:3]  = e.pos
                        self.traj4_pos = np.array([evals[:,0], evals[:,1]])
                        trajectory, = self.ax.plot3D(evals[:,0], evals[:,1], evals[:,2], color = 'g', alpha=0.2)
                        trajectory.set_visible(True)
                        self.traj_lst.append(trajectory)
                    i += 1
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
            self.cf1_x = float(msg.pose.position.x)
            self.cf1_y = float(msg.pose.position.y)
            self.cf1_z = float(msg.pose.position.z)
            self.cf1_p = np.array([self.cf1_x, self.cf1_y], dtype=float)
            self.cf1_x_data.append(self.cf1_x)
            self.cf1_y_data.append(self.cf1_y) 
            self.cf1_z_data.append(self.cf1_z)
    
    def cf2_callback(self, msg):
        if isinstance(msg.pose.position.x, float):
            self.cf2_x = float(msg.pose.position.x)
            self.cf2_y = float(msg.pose.position.y)
            self.cf2_z = float(msg.pose.position.z)
            self.cf2_p = np.array([self.cf2_x, self.cf2_y])
            self.cf2_x_data.append(self.cf2_x)
            self.cf2_y_data.append(self.cf2_y) 
            self.cf2_z_data.append(self.cf2_z)

    def cf3_callback(self, msg):
        if isinstance(msg.pose.position.x, float):
            self.cf3_x = float(msg.pose.position.x)
            self.cf3_y = float(msg.pose.position.y)
            self.cf3_z = float(msg.pose.position.z)
            self.cf3_p = np.array([self.cf3_x, self.cf3_y])
            self.cf3_x_data.append(self.cf3_x)
            self.cf3_y_data.append(self.cf3_y) 
            self.cf3_z_data.append(self.cf3_z)
        
    def cf4_callback(self, msg):
        if isinstance(msg.pose.position.x, float):
            self.cf4_x = float(msg.pose.position.x)
            self.cf4_y = float(msg.pose.position.y)
            self.cf4_z = float(msg.pose.position.z)
            self.cf4_p = np.array([self.cf4_x, self.cf4_y])
            self.cf4_x_data.append(self.cf4_x)
            self.cf4_y_data.append(self.cf4_y) 
            self.cf4_z_data.append(self.cf4_z)

    def update_plot(self, frame):
        if len(self.cf1_x_data) > 0:
            self.cf1_log.set_data(self.cf1_x_data, self.cf1_y_data)
            self.cf1_log.set_3d_properties(self.cf1_z_data)
            self.cf1_pos.set_data(self.cf1_x_data[-1], self.cf1_y_data[-1])
            self.cf1_pos.set_3d_properties(self.cf1_z_data[-1])
            if len(self.traj1_pos[0]) > 2:
                self.distance1.append(min(np.linalg.norm(self.traj1_pos.T-self.cf1_p, axis=1)))
                if self.distance1[-1] > 1 and not self.emergency_active:
                    print("Warning, drone1 error > 1 m (" + str(self.distance1[-1]) + " m)!")
                    print("Emergency landing")
                    self.emergency()
                    self.emergency_active = True

        if len(self.cf2_x_data) > 0:
            self.cf2_log.set_data(self.cf2_x_data, self.cf2_y_data)
            self.cf2_log.set_3d_properties(self.cf2_z_data)
            self.cf2_pos.set_data(self.cf2_x_data[-1], self.cf2_y_data[-1])
            self.cf2_pos.set_3d_properties(self.cf2_z_data[-1])
            if len(self.traj2_pos[0]) > 2:    
                self.distance2.append(min(np.linalg.norm(self.traj2_pos.T-self.cf2_p, axis=1)))
                if self.distance2[-1] > 1 and not self.emergency_active:
                    print("Warning, drone2 error > 1 m (" + str(self.distance2[-1]) + " m)!")
                    print("Emergency landing")
                    self.emergency()
                    self.emergency_active = True
        if len(self.cf3_x_data) > 0:
            self.cf3_log.set_data(self.cf3_x_data, self.cf3_y_data)
            self.cf3_log.set_3d_properties(self.cf3_z_data)
            self.cf3_pos.set_data(self.cf3_x_data[-1], self.cf3_y_data[-1])
            self.cf3_pos.set_3d_properties(self.cf3_z_data[-1])
            if len(self.traj3_pos[0]) > 2:
                self.distance3.append(min(np.linalg.norm(self.traj3_pos.T-self.cf3_p, axis=1)))
                if self.distance3[-1] > 1 and not self.emergency_active:
                    print("Warning, drone3 error > 1 m (" + str(self.distance3[-1]) + " m)!")
                    print("Emergency landing")
                    self.emergency()
                    self.emergency_active = True
        if len(self.cf4_x_data) > 0:
            self.cf4_log.set_data(self.cf4_x_data, self.cf4_y_data)
            self.cf4_log.set_3d_properties(self.cf4_z_data)
            self.cf4_pos.set_data(self.cf4_x_data[-1], self.cf4_y_data[-1])
            self.cf4_pos.set_3d_properties(self.cf4_z_data[-1])
            if len(self.traj4_pos[0]) > 2:
                self.distance4.append(min(np.linalg.norm(self.traj4_pos.T-self.cf4_p, axis=1)))
                if self.distance1[-4] > 1 and not self.emergency_active:
                    print("Warning, drone4 error > 1 m (" + str(self.distance4[-1]) + " m)!")
                    print("Emergency landing")
                    self.emergency()
                    self.emergency_active = True

    def save_log(self, name):
        plt.savefig("./Log_files/" + str(name) + ".eps")
        np.savetxt("./Log_files/cf1_recordedPosition.csv", np.array([self.cf1_x_data, self.cf1_y_data, self.cf1_z_data]).T, delimiter = ',', fmt = '%10f')
        np.savetxt("./Log_files/cf2_recordedPosition.csv", np.array([self.cf2_x_data, self.cf2_y_data, self.cf2_z_data]).T, delimiter = ',', fmt = '%10f')
        np.savetxt("./Log_files/cf3_recordedPosition.csv", np.array([self.cf3_x_data, self.cf3_y_data, self.cf3_z_data]).T, delimiter = ',', fmt = '%10f')
        np.savetxt("./Log_files/cf4_recordedPosition.csv", np.array([self.cf4_x_data, self.cf4_y_data, self.cf4_z_data]).T, delimiter = ',', fmt = '%10f')

    def plot_error(self):
        self.fig2, self.ax2 = plt.subplots()
        if len(self.distance1) > 0:
            t = np.linspace(0,len(self.distance1), num = len(self.distance1))
            self.ax2.plot(t,self.distance1, color ="black", label = "cf1")
        if len(self.distance2) > 0:
            t = np.linspace(0,len(self.distance2), num = len(self.distance2))
            self.ax2.plot(t,self.distance2, color ="blue", label = "cf2")
        if len(self.distance3) > 0:
            t = np.linspace(0,len(self.distance3), num = len(self.distance3))
            self.ax2.plot(t,self.distance3, color ="red", label = "cf3")
        if len(self.distance4) > 0:
            t = np.linspace(0,len(self.distance4), num = len(self.distance4))
            self.ax2.plot(t,self.distance4, color ="green", label = "cf4")

        self.ax2.set_title("Distance to trajectory")
        self.ax2.set_xlabel("Time")
        self.ax2.set_ylabel("Error from reference")
        self.ax2.legend()
        self.ax2.grid()
        self.fig2.savefig("./Flight_log_files/Path_error.png")
        plt.show()

if __name__ == "__main__":
    vis = Visualisation()

    rospy.init_node('Pose_Listener')
    cf1 = rospy.Subscriber("/qualisys/cf1/pose", PoseStamped, vis.cf1_callback)
    cf2 = rospy.Subscriber("/qualisys/cf2/pose", PoseStamped, vis.cf2_callback)
    cf3 = rospy.Subscriber("/qualisys/cf3/pose", PoseStamped, vis.cf3_callback)
    cf4 = rospy.Subscriber("/qualisys/cf4/pose", PoseStamped, vis.cf4_callback)

    ani = animation.FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init, interval = 50) 
    
    mainwindow = Tkinter.Tk()
    mainwindow.title("CrazyTrain - Realtime simulation control")
    mainwindow.geometry("350x300")
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
    emergency_bttn = Tkinter.Button(frame, text = "EMERGENCY",  bg='red', command = lambda : vis.emergency()) 
    emergency_bttn.pack()
    plot = Tkinter.Button(frame, text = "Plot error",  bg='grey', command = lambda : [vis.plot_error(), vis.Sim_Paus(ani)]) 
    plot.pack()
    plt.show()