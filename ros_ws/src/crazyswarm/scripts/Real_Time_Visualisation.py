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

class Visualisation:
    def __init__(self):
        # Creating figure window
        self.fig = plt.figure()
        self.ax = Axes3D(self.fig)
        self.anim_running = True
        self.save = False
        self.traj_on = False

        self.cf1_x_data, self.cf1_y_data, self.cf1_z_data = [],[],[]
        self.cf2_x_data, self.cf2_y_data, self.cf2_z_data = [],[],[]
        self.cf3_x_data, self.cf3_y_data, self.cf3_z_data = [],[],[]
        self.cf4_x_data, self.cf4_y_data, self.cf4_z_data = [],[],[]

        self.cf1_log, = self.ax.plot3D(self.cf1_x_data[0:1], self.cf1_y_data[0:1], self.cf1_z_data[0:1], c='black', linestyle='dotted') # self.ax.plot3D([], [], [], c='black', linestyle='dotted') #    # 
        self.cf2_log, = self.ax.plot3D(self.cf2_x_data[0:1], self.cf2_y_data[0:1], self.cf2_z_data[0:1], c='blue', linestyle='dotted')
        self.cf3_log, = self.ax.plot3D(self.cf3_x_data[0:1], self.cf3_y_data[0:1], self.cf3_z_data[0:1], c='red', linestyle='dotted')
        self.cf4_log, = self.ax.plot3D(self.cf4_x_data[0:1], self.cf4_y_data[0:1], self.cf4_z_data[0:1], c='green', linestyle='dotted')
        
        self.cf1_pos, = self.ax.plot3D([], [], [], 'ko', label = "Cf1") #self.cf1_x_data[0:1], self.cf1_y_data[0:1], self.cf1_z_data[0:1]
        self.cf2_pos, = self.ax.plot3D([], [], [], 'bo', label = "Cf2")
        self.cf3_pos, = self.ax.plot3D([], [], [], 'ro', label = "Cf3")
        self.cf4_pos, = self.ax.plot3D([], [], [], 'go', label = "Cf4")

        self.distance1 = []
        self.distance2 = []
        self.distance3 = []
        self.distance4 = []

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
            for traj in self.traj_lst:
                traj.set_visible(False)
                #traj.pop(0).remove() eventuellt
            self.traj_on = False

        else:
            self.traj_lst = []
            self.traj_pos = []
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
                    self.traj1_pos = np.array([evals[:,0], evals[:,1], evals[:,2]])
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
                    self.traj2_pos = np.array([evals[:,0], evals[:,1], evals[:,2]]) 
                    trajectory, = self.ax.plot3D(evals[:,0], evals[:,1], evals[:,2], color = 'b', alpha=0.2)

                    if("drone3" in file):
                        self.traj3 = uav_trajectory.Trajectory()
                        self.traj3.loadcsv(file)
                        ts = np.arange(0, self.traj3.duration, 0.01)
                        evals = np.empty((len(ts), 15))
                        for t, k in zip(ts, range(0, len(ts))):
                            e = self.traj3.eval(t)
                            evals[k, 0:3]  = e.pos
                    self.traj3_pos = np.array([evals[:,0], evals[:,1], evals[:,2]])
                    trajectory, = self.ax.plot3D(evals[:,0], evals[:,1], evals[:,2], color = 'r', alpha=0.2)

                    if("drone4" in file):
                        self.traj4 = uav_trajectory.Trajectory()
                        self.traj4.loadcsv(file)
                        ts = np.arange(0, self.traj4.duration, 0.01)
                        evals = np.empty((len(ts), 15))
                        for t, k in zip(ts, range(0, len(ts))):
                            e = self.traj4.eval(t)
                            evals[k, 0:3]  = e.pos
                    self.traj4_pos = np.array([evals[:,0], evals[:,1], evals[:,2]])
                    trajectory, = self.ax.plot3D(evals[:,0], evals[:,1], evals[:,2], color = 'g', alpha=0.2)
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
        self.cf1_x = msg.pose.position.x
        self.cf1_y = msg.pose.position.y
        self.cf1_z = msg.pose.position.z
        self.cf1_p = np.array([self.cf1_x, self.cf1_y, self.cf1_z])
        self.cf1_x_data.append(self.cf1_x)
        self.cf1_y_data.append(self.cf1_y) 
        self.cf1_z_data.append(self.cf1_z)
    
    def cf2_callback(self, msg):
        self.cf2_x = msg.pose.position.x
        self.cf2_y = msg.pose.position.y
        self.cf2_z = msg.pose.position.z
        self.cf2_p = np.array([self.cf2_x, self.cf2_y, self.cf2_z])
        self.cf2_x_data.append(self.cf2_x)
        self.cf2_y_data.append(self.cf2_y) 
        self.cf2_z_data.append(self.cf2_z)

    def cf3_callback(self, msg):
        self.cf3_x = msg.pose.position.x
        self.cf3_y = msg.pose.position.y
        self.cf3_z = msg.pose.position.z
        self.cf3_p = np.array([self.cf3_x, self.cf3_y, self.cf3_z])
        self.cf3_x_data.append(self.cf3_x)
        self.cf3_y_data.append(self.cf3_y) 
        self.cf3_z_data.append(self.cf3_z)
    
    def cf4_callback(self, msg):
        self.cf4_x = msg.pose.position.x
        self.cf4_y = msg.pose.position.y
        self.cf4_z = msg.pose.position.z
        self.cf4_p = np.array([self.cf4_x, self.cf4_y, self.cf4_z])
        self.cf4_x_data.append(self.cf4_x)
        self.cf4_y_data.append(self.cf4_y) 
        self.cf4_z_data.append(self.cf4_z)

    def update_plot(self, frame):

        self.cf1_log.set_data(self.cf1_x_data[:frame], self.cf1_y_data[:frame])
        self.cf1_log.set_3d_properties(self.cf1_z_data[:frame])
        self.cf2_log.set_data(self.cf2_x_data[:frame], self.cf2_y_data[:frame])
        self.cf2_log.set_3d_properties(self.cf2_z_data[:frame])
        self.cf3_log.set_data(self.cf3_x_data[:frame], self.cf3_y_data[:frame])
        self.cf3_log.set_3d_properties(self.cf3_z_data[:frame])
        self.cf4_log.set_data(self.cf4_x_data[:frame], self.cf4_y_data[:frame])
        self.cf4_log.set_3d_properties(self.cf4_z_data[:frame])


        self.cf1_pos.set_data(self.cf1_x_data[frame], self.cf1_y_data[frame])
        self.cf1_pos.set_3d_properties(self.cf1_z_data[frame])
        self.cf2_pos.set_data(self.cf2_x_data[frame], self.cf2_y_data[frame])
        self.cf2_pos.set_3d_properties(self.cf2_z_data[frame])
        self.cf3_pos.set_data(self.cf3_x_data[frame], self.cf3_y_data[frame])
        self.cf3_pos.set_3d_properties(self.cf3_z_data[frame])
        self.cf4_pos.set_data(self.cf4_x_data[frame], self.cf4_y_data[frame])
        self.cf4_pos.set_3d_properties(self.cf4_z_data[frame])

        self.distance1.append(min(np.linalg.norm(self.traj1_pos.T-self.cf1_p, axis=1)))
        self.distance2.append(min(np.linalg.norm(self.traj2_pos.T-self.cf2_p, axis=1)))
        self.distance3.append(min(np.linalg.norm(self.traj3distance3_pos.T-self.cf3distance3_p, axis=1)))
        self.distance4.append(min(np.linalg.norm(self.traj4_pos.T-self.cf4_p, axis=1)))

    def save_log(self, name):
        plt.savefig("./Log_files/" + str(name) + ".png")
        np.savetxt("./Log_files/cf1_recordedPosition.csv", np.array([self.cf1_x_data, self.cf1_y_data, self.cf1_z_data]).T, delimiter = ',', fmt = '%10f')
        np.savetxt("./Log_files/cf2_recordedPosition.csv", np.array([self.cf2_x_data, self.cf2_y_data, self.cf2_z_data]).T, delimiter = ',', fmt = '%10f')
        np.savetxt("./Log_files/cf3_recordedPosition.csv", np.array([self.cf3_x_data, self.cf3_y_data, self.cf3_z_data]).T, delimiter = ',', fmt = '%10f')
        np.savetxt("./Log_files/cf4_recordedPosition.csv", np.array([self.cf4_x_data, self.cf4_y_data, self.cf4_z_data]).T, delimiter = ',', fmt = '%10f')


if __name__ == "__main__":
    vis = Visualisation()

    rospy.init_node('Pose_Listener')
    cf1 = rospy.Subscriber("/qualisys/cf1/pose", PoseStamped, vis.cf1_callback)
    cf2 = rospy.Subscriber("/qualisys/cf2/pose", PoseStamped, vis.cf2_callback)
    cf3 = rospy.Subscriber("/qualisys/cf3/pose", PoseStamped, vis.cf3_callback)
    cf4 = rospy.Subscriber("/qualisys/cf4/pose", PoseStamped, vis.cf4_callback)

    ani = animation.FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init, interval = 50) # oklart om true eller ej!
    
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
    plt.show()