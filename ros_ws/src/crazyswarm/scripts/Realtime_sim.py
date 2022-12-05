import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection # Oklart om dessa behövs! Prova att ta bort vid nästa test
from mpl_toolkits.mplot3d.art3d import Line3DCollection # -||-
import matplotlib.animation as animation
import uav_trajectory
import tkinter as Tkinter
import os
import csv
from pycrazyswarm.visualizer import visMatplotlib
import sys

class Visualisation:
    def __init__(self):
        # Creating figure window
        self.fig = plt.figure()
        self.ax = Axes3D(self.fig)
        self.anim_running = True
        self.save = False
        self.traj_on = False

        # Initialising data log lists for all cfs
        self.cf1_x_data, self.cf1_y_data, self.cf1_z_data = [], [], []
        self.cf2_x_data, self.cf2_y_data, self.cf2_z_data = [], [], []
        self.cf3_x_data, self.cf3_y_data, self.cf3_z_data = [], [], []
        self.cf4_x_data, self.cf4_y_data, self.cf4_z_data = [], [], []

        # Initialising plot for past data (dotted lines)
        self.cf1_log, = self.ax.plot(self.cf1_x_data[0:1], self.cf1_y_data[0:1], self.cf1_z_data[0:1], c='black', linestyle='dotted') # self.ax.plot3D([], [], [], c='black', linestyle='dotted') #    # 
        self.cf2_log, = self.ax.plot(self.cf2_x_data[0:1], self.cf2_y_data[0:1], self.cf2_z_data[0:1], c='blue', linestyle='dotted')
        self.cf3_log, = self.ax.plot(self.cf3_x_data[0:1], self.cf3_y_data[0:1], self.cf3_z_data[0:1], c='red', linestyle='dotted')
        self.cf4_log, = self.ax.plot(self.cf4_x_data[0:1], self.cf4_y_data[0:1], self.cf4_z_data[0:1], c='green', linestyle='dotted')
        
        # Initialising plot for current position
        self.cf1_pos, = self.ax.plot([], [], [], 'ko', label = "Cf1")
        self.cf2_pos, = self.ax.plot([], [], [], 'bo', label = "Cf2")
        self.cf3_pos, = self.ax.plot([], [], [], 'ro', label = "Cf3")
        self.cf4_pos, = self.ax.plot([], [], [], 'go', label = "Cf4")

        # Initialising lists to store distances from trajectories 
        self.distance1, self.distance2, self.distance3 , self.distance4 = [], [], [], []
        self.traj_lst = []
        self.traj1_pos = np.empty([1,2],dtype=float)
        self.traj2_pos = np.empty([1,2],dtype=float)
        self.traj3_pos = np.empty([1,2],dtype=float)
        self.traj4_pos = np.empty([1,2],dtype=float)
        # Bool for checking if emergency function has been called. 
        self.emergency_active = False

    def plot_init(self):
        # Setting up plot limits and axes
        self.ax.set_xlim3d(-5, 5)
        self.ax.set_ylim3d(-5, 5)
        self.ax.set_zlim3d(0, 2.5)
        self.ax.set_xlabel('Position X [m]')
        self.ax.set_ylabel('Position Y [m]')
        self.ax.set_zlabel('Position Z [m]')
        self.ax.set_title("Drone real-time position")
        self.ax.legend()

    def emergency(self):
        # Calls emergency function which terminates all terminals and makes all drones land 
        os.system('gnome-terminal -- bash ../../../../../GUI/bash_scripts/emergency.sh')

    def Show_traj(self): 
        # Function that plots or removes all drone trajectories. Searches for all trajectory files among the script files 
        # and assigns corresponding trajectory to each drone. 
        if self.traj_on:
            for traj in self.traj_lst:
                #traj.set_visible(False)
                traj.pop(0).remove() # Removes all trajectories from plot.
            self.traj_on = False
        else:
            self.traj_lst = []
            path = "."
            trajectory_files = [f for f in os.listdir(path) if os.path.isfile(os.path.join(path, f))]
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
            self.traj_on = True

    def Show_obstacles(self):
        sys.path.append("/home/crazycrowd/CrazyTrain/CrazyTrain2022/crazyswarm/ros_ws/src/crazyswarm/scripts/pycrazyswarm/visualizer")
        file_obs = "obstacles.csv"
        with open(file_obs, 'r') as file:
            reader = csv.reader(file, skipinitialspace=True)
            obs = np.empty((0,6),int)
            for coord in reader:
                step = np.array([[float(coord[0]),float(coord[1]),float(coord[2]),float(coord[3]),float(coord[4]),float(coord[5])]])
                obs = np.append(obs ,step, axis = 0)
        for i in range(0,len(obs)):
            visMatplotlib.add_obs(self.ax, obs[i][0], obs[i][1], obs[i][2], obs[i][3], obs[i][4], obs[i][5])

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
            self.cf1_x, self.cf1_y, self.cf1_z = float(msg.pose.position.x), float(msg.pose.position.y), float(msg.pose.position.z)
            self.cf1_p = np.array([self.cf1_x, self.cf1_y], dtype=float)
            self.cf1_x_data.append(self.cf1_x)
            self.cf1_y_data.append(self.cf1_y) 
            self.cf1_z_data.append(self.cf1_z)
    
    def cf2_callback(self, msg):
        if isinstance(msg.pose.position.x, float):
            self.cf2_x, self.cf2_y, self.cf2_z = float(msg.pose.position.x), float(msg.pose.position.y), float(msg.pose.position.z)
            self.cf2_p = np.array([self.cf2_x, self.cf2_y])
            self.cf2_x_data.append(self.cf2_x)
            self.cf2_y_data.append(self.cf2_y) 
            self.cf2_z_data.append(self.cf2_z)

    def cf3_callback(self, msg):
        if isinstance(msg.pose.position.x, float):
            self.cf3_x, self.cf3_y, self.cf3_z = float(msg.pose.position.x), float(msg.pose.position.y), float(msg.pose.position.z)
            self.cf3_p = np.array([self.cf3_x, self.cf3_y])
            self.cf3_x_data.append(self.cf3_x)
            self.cf3_y_data.append(self.cf3_y) 
            self.cf3_z_data.append(self.cf3_z)
        
    def cf4_callback(self, msg):
        if isinstance(msg.pose.position.x, float):
            self.cf4_x, self.cf4_y , self.cf4_z  = float(msg.pose.position.x), float(msg.pose.position.y), float(msg.pose.position.z)
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
                    print("Emergency landing!")
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
                    print("Emergency landing!")
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
                    print("Emergency landing!")
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
                    print("Emergency landing!")
                    self.emergency()
                    self.emergency_active = True

    def save_log(self, name):
        plt.savefig("./Log_files/" + str(name) + ".png")
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
        self.fig2.savefig("./Log_files/Path_error.png")
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
    mainwindow.geometry("400x300")
    frame = Tkinter.Frame(mainwindow)  #inside box
    frame.pack()
    start_print = Tkinter.Button(frame, text = "Start/Pause visualisation",  bg='green', command = lambda : vis.Sim_Paus(ani))
    start_print.pack()
    traj_print = Tkinter.Button(frame, text="Show trajectory", bg='grey', command = lambda : vis.Show_traj())
    traj_print.pack()
    show_obstacles = Tkinter.Button(frame, text="Show obstacles", bg='grey', command = lambda : vis.Show_obstacles())
    show_obstacles.pack()
    entry1 = Tkinter.Entry(mainwindow)
    entry1.insert(0, "Name of saved file")
    entry1.pack()
    save_print = Tkinter.Button(frame, text = "Save plot to file",  bg='grey', command = lambda : vis.save_log(entry1.get())) 
    save_print.pack()
    plot = Tkinter.Button(frame, text = "Plot error",  bg='grey', command = lambda : [vis.plot_error(), vis.Sim_Paus(ani)]) 
    plot.pack()
    emergency_bttn = Tkinter.Button(frame, text = "EMERGENCY",  bg='red', command = lambda : vis.emergency()) 
    emergency_bttn.pack()
    plt.show()