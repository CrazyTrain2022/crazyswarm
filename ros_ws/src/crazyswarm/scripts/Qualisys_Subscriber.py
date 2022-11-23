import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, TransformStamped
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import uav_trajectory
import tkinter as Tkinter
import os

class Visualisation:
    def __init__(self):
        # Creating figure window
        self.fig = plt.figure()
        self.ax = Axes3D(self.fig)
        self.anim_running = True
    def plot_init(self):
        # Setting up plot 
        self.ax.set_xlim3d(-5, 5)
        self.ax.set_ylim3d(-5, 5)
        self.ax.set_zlim3d(0, 3)
        self.ax.set_xlabel('Position X [m]')
        self.ax.set_ylabel('Position Y [m]')
        self.ax.set_zlabel('Position Z [m]')
        self.ax.set_title('Drone real-time position')
        self.ax.legend()
        # return self.ln

    def Show_traj(self): # Function that plots all drone trajectories
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

                trajectory = self.ax.plot3D(evals[:,0], evals[:,1], evals[:,2], color = 'b')
                i += 1
        
    def Sim_Paus(self):
        if self.anim_running:
            ani.event_source.stop()
            self.anim_running = False
            print("Animation paused")
        else:
            ani.event_source.start()
            self.anim_running = True
            print("Animation resumed")

class cfs:
    def __init__(self, vis):
        self.save = False
        self.cf1_log = vis.ax.plot3D([], [], [], c='black', linestyle='dotted')
        self.cf2_log = vis.ax.plot3D([], [], [], c='blue', linestyle='dotted')
        self.cf3_log = vis.ax.plot3D([], [], [], c='red', linestyle='dotted')
        self.cf4_log = vis.ax.plot3D([], [], [], c='green', linestyle='dotted')
        
        self.cf1_pos = vis.ax.scatter([], [], [], 'ko', label = "Cf1")
        self.cf2_pos = vis.ax.scatter([], [], [], 'bo', label = "Cf2")
        self.cf3_pos = vis.ax.scatter([], [], [], 'ro', label = "Cf3")
        self.cf4_pos = vis.ax.scatter([], [], [], 'go', label = "Cf4")

        self.cf1_x_data, self.cf1_y_data, self.cf1_z_data = [],[],[]
        self.cf2_x_data, self.cf2_y_data, self.cf2_z_data = [],[],[]
        self.cf3_x_data, self.cf3_y_data, self.cf3_z_data = [],[],[]
        self.cf4_x_data, self.cf4_y_data, self.cf4_z_data = [],[],[]

    def cf1_callback(self, msg):
        self.cf1_x = msg.pose.position.x
        self.cf1_y = msg.pose.position.y
        self.cf1_z = msg.pose.position.z
        self.cf1_x_data.append(self.cf1_x)
        self.cf1_y_data.append(self.cf1_y) 
        self.cf1_z_data.append(self.cf1_z)
    
    def cf2_callback(self, msg):
        self.cf2_x = msg.pose.position.x
        self.cf2_y = msg.pose.position.y
        self.cf2_z = msg.pose.position.z
        self.cf2_x_data.append(self.cf2_x)
        self.cf2_y_data.append(self.cf2_y) 
        self.cf2_z_data.append(self.cf2_z)
    def cf3_callback(self, msg):
        self.cf3_x = msg.pose.position.x
        self.cf3_y = msg.pose.position.y
        self.cf3_z = msg.pose.position.z
        self.cf3_x_data.append(self.cf3_x)
        self.cf3_y_data.append(self.cf3_y) 
        self.cf3_z_data.append(self.cf3_z)
    
    def cf4_callback(self, msg):
        self.cf4_x = msg.pose.position.x
        self.cf4_y = msg.pose.position.y
        self.cf4_z = msg.pose.position.z
        self.cf4_x_data.append(self.cf4_x)
        self.cf4_y_data.append(self.cf4_y) 
        self.cf4_z_data.append(self.cf4_z)

    def update_plot(self, frame):
        vis.ax.clear() # Oklart om detta funkar...
        self.cf1_log._offsets3d = [self.cf1_x_data, self.cf1_y_data, self.cf1_z_data]
        self.cf2_log._offsets3d = [self.cf2_x_data, self.cf2_y_data, self.cf2_z_data]
        self.cf3_log._offsets3d = [self.cf3_x_data, self.cf3_y_data, self.cf3_z_data]
        self.cf4_log._offsets3d = [self.cf4_x_data, self.cf4_y_data, self.cf4_z_data]

        self.cf1_pos.set_data(self.cf1_x, self.cf1_y, self.cf1_z)
        self.cf2_pos.set_data(self.cf2_x, self.cf2_y, self.cf2_z)
        self.cf3_pos.set_data(self.cf3_x, self.cf3_y, self.cf3_z)
        self.cf4_pos.set_data(self.cf4_x, self.cf4_y, self.cf4_z)
    
    def save_log(self):
        np.savetxt("./Log_files/cf1_recordedPosition.csv", np.array([self.cf1_x_data, self.cf1_y_data, self.cf1_z_data]).T, delimiter = ',', fmt = '%10f')
        np.savetxt("./Log_files/cf2_recordedPosition.csv", np.array([self.cf2_x_data, self.cf2_y_data, self.cf2_z_data]).T, delimiter = ',', fmt = '%10f')
        np.savetxt("./Log_files/cf3_recordedPosition.csv", np.array([self.cf3_x_data, self.cf3_y_data, self.cf3_z_data]).T, delimiter = ',', fmt = '%10f')
        np.savetxt("./Log_files/cf4_recordedPosition.csv", np.array([self.cf4_x_data, self.cf4_y_data, self.cf4_z_data]).T, delimiter = ',', fmt = '%10f')

if __name__ == "__main__":
    vis = Visualisation()

    mainwindow = Tkinter.Tk()
    mainwindow.title("CrazyTrain - Realtime simulation control")
    mainwindow.geometry("350x300")
    frame = Tkinter.Frame(mainwindow)  #inside box
    frame.pack()
    start_print = Tkinter.Button(frame, text = "Start/Pause visualisation",  bg='green', command = Visualisation.Sim_Paus)
    start_print.pack()
    traj_print = Tkinter.Button(frame, text="Show trajectory", bg='grey', command = vis.Show_traj)
    traj_print.pack()
    start_print = Tkinter.Button(frame, text = "Save position to file",  bg='grey', command = cfs.save_log) # variable = bool_save, onvalue=0, offvalue=1, command=save_csv)
    start_print.pack()

    cf = cfs(vis)
    rospy.init_node('Pose_Listener')
    cf1 = rospy.Subscriber("/qualisys/cf1/pose", PoseStamped, cf.cf1_callback)
    cf2 = rospy.Subscriber("/qualisys/cf2/pose", PoseStamped, cf.cf2_callback)
    cf3 = rospy.Subscriber("/qualisys/cf3/pose", PoseStamped, cf.cf3_callback)
    cf4 = rospy.Subscriber("/qualisys/cf4/pose", PoseStamped, cf.cf4_callback)

    ani = animation.FuncAnimation(vis.fig, cf.update_plot, init_func=vis.plot_init, blit=True) # oklart om true eller ej!
    plt.show()