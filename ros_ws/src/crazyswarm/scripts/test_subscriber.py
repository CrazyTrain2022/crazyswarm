#!/usr/bin/env python
import rospy
import numpy as np
from pycrazyswarm import *
from geometry_msgs.msg import PoseStamped, TransformStamped
from pycrazyswarm import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import matplotlib.animation as animation
import uav_trajectory
import os
import tkinter as Tkinter

def Sim_Stop():
    global stop
    stop = True

def Show_traj():
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


            trajectory = ax.plot3D(evals[:,0], evals[:,1], evals[:,2])
            i += 1


def Simulate(data):
    # rospy.loginfo(data.pose.position.x)
    rospy.loginfo(data.transform.translation.x)
    print("Time since birth of Kung Carl Gustav XVI:", data.header.stamp.secs)
    cf = data.child_frame_id
    time = data.header.stamp.secs
    k = 0
    for cf in allcfs.crazyflies:
        name = "cf" + str(k+1)
        x_pos = data.transform.translation.x
        y_pos = data.transform.translation.y
        z_pos = data.transform.translation.z
        log_x[k].append(x_pos)
        log_y[k].append(y_pos)
        log_z[k].append(z_pos)
        ax.scatter(x_pos, y_pos, z_pos, marker = 'o')

        dataLog = np.array([log_x[k], log_y[k], log_z[k]])
        
        ax.plot3D(dataLog[0], dataLog[1], 
                dataLog[2], c='black',linestyle='dotted')

        #Position_log.append(current_pos)

        print("X Position:" , x_pos)
        print("Y Position:" , y_pos)
        print("Z Position:" , z_pos)
        print("------------------------------------")
    
    ax.set_title('Drone real-time position')
    ax.axes.set_xlim3d(left= -3, right = 3)
    ax.axes.set_ylim3d(bottom = -3, top = 3)
    ax.axes.set_zlim3d(bottom = 0, top = 2)

def listner():
    global stop
    if stop:
        return 0 # eventuellt break?
    rospy.init_node('Pose_Listner', anonymous=True)
    rospy.Subscriber("/tf", TransformStamped, Simulate)
    # rospy.Subscriber("/cf1/pose", PoseStamped, Simulate)
    rospy.spin()

if __name__ == '__main__':

    stop = False
    swarm = Crazyswarm()
    allcfs = swarm.allcfs
    log_x = []
    log_y = []
    log_z = []
    for cf in allcfs.crazyflies:
        log_x.append([])
        log_y.append([])
        log_z.append([])
    color = ['b', 'r', 'g', 'k']
    fig = plt.figure(1)
    ax = Axes3D(fig)
    # ax.set_xlabel('Position X')
    # ax.set_ylabel('Position Y')
    # ax.set_zlabel('Position Z')
    # ax.set_title('Drone real-time position')
    # ax.axes.set_xlim3d(left= -5, right = 5)
    # ax.axes.set_ylim3d(bottom = -5, top = 5)
    # ax.axes.set_zlim3d(bottom = 0, top = 2)

    # Simulation GUI
    mainwindow = Tkinter.Tk()
    mainwindow.title("CrazyTrain - Realtime simulation control")
    mainwindow.geometry("350x300")

    frame = Tkinter.Frame(mainwindow)  #inside box
    frame.pack()

    start_print = Tkinter.Button(frame, text = "Start/Pause visualisation",  bg='green', command = Sim_Stop)
    start_print.pack()
    Show_traj()
    Position_log = np.array([])
    listner() 
    
    plt.show()

