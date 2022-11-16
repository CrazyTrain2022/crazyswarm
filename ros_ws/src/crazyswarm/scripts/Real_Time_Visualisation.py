from test_subscriber import *
from pycrazyswarm import *
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import uav_trajectory
import os
import tkinter as Tkinter

def save_csv():
    global bool_save
    if bool_save == 1:
        bool_save = 0
    elif bool_save == 0:
        bool_save = 1


def Sim_Paus():
    global anim_running
    if anim_running:
        anim.event_source.stop()
        anim_running = False
        print("Animation paused")
    else:
        anim.event_source.start()
        anim_running = True
        print("Animation resumed")

def animate_func(i):
    color = ['b', 'r', 'g', 'k']
    ax.clear()
    k = 0
    for cf in vis_allcfs.crazyflies:
        print("running")
        name = "cf" + str(k+1)
        cf_pos = cf.position()
        log_x[k].append(cf_pos[0])
        log_y[k].append(cf_pos[1])
        log_z[k].append(cf_pos[2])

        dataLog = np.array([log_x[k], log_y[k], log_z[k]])
        
        ax.plot3D(dataLog[0], dataLog[1], 
                dataLog[2], c='black',linestyle='dotted')
        
        log_x.append(cf_pos[0])
        log_y.append(cf_pos[1])
        log_z.append(cf_pos[2])
        
        ax.scatter(cf_pos[0], cf_pos[1], cf_pos[2], c= color[k], label = name)
        ax.legend()
        if bool_save:
            print("Saving log data to file: " + name + "recordedPosition.csv")
            np.savetxt("./Log_files/" + name + "recordedPosition.csv", dataLog.T, delimiter = ',', fmt = '%10f')

        k+=1
    
    
    #ax.scatter(cf2_pos[0], cf2_pos[1], cf2_pos[2], c= 'k')
    # ax.set_xlabel('Position X')
    # ax.set_ylabel('Position Y')
    # ax.set_zlabel('Position Z')
    # ax.set_title('Drone real-time position')
    ax.axes.set_xlim3d(left= -5, right = 5)
    ax.axes.set_ylim3d(bottom = -5, top = 5)
    ax.axes.set_zlim3d(bottom = 0, top = 2)


if __name__ == "__main__":
    mainwindow = Tkinter.Tk()
    mainwindow.title("CrazyTrain - Realtime simulation control")
    mainwindow.geometry("350x300")

    frame = Tkinter.Frame(mainwindow)  #inside box
    frame.pack()

    # traj_print = Tkinter.Checkbutton(frame, text="Show trajectory" )
    # traj_print.pack()
    # start_print=Tkinter.Button(frame, text = "Start trajectory",  bg='green')
    # start_print.pack()
    start_print = Tkinter.Button(frame, text = "Start/Pause visualisation",  bg='green', command = Sim_Paus)
    start_print.pack()
    start_print = Tkinter.Button(frame, text = "Save position to file",  bg='grey', command = save_csv) # variable = bool_save, onvalue=0, offvalue=1, command=save_csv)
    start_print.pack()

    # traj_lst = []
    # i = 0
    # path = "."
    # trajectory_files = [f for f in os.listdir(path) if os.path.isfile(os.path.join(path, f))]
    # for file in trajectory_files:
    #     if("trajectory.csv" in file):
    #         traj_lst.append(uav_trajectory.Trajectory())
    #         traj_lst[i].loadcsv(file)
    #         i += 1


    anim_running = True
    vis_swarm = Crazyswarm()
    vis_allcfs = vis_swarm.allcfs
    log_x = []
    log_y = []
    log_z = []
    for cf in vis_allcfs.crazyflies:
        log_x.append([])
        log_y.append([])
        log_z.append([])

    fig = plt.figure()
    ax = Axes3D(fig)
    ax.set_xlabel('Position X')
    ax.set_ylabel('Position Y')
    ax.set_zlabel('Position Z')
    ax.set_title('Drone real-time position')
    ax.axes.set_xlim3d(left= -5, right = 5)
    ax.axes.set_ylim3d(bottom = -5, top = 5)
    ax.axes.set_zlim3d(bottom = 0, top = 2)

    anim = animation.FuncAnimation(fig, animate_func, interval=500, blit=False)
    plt.show()
