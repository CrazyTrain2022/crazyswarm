from test_subscriber import *
from pycrazyswarm import *
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import uav_trajectory
import tkinter as Tkinter

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
    
    for cf in allcfs.crazyflies:

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
    
    
    #ax.scatter(cf2_pos[0], cf2_pos[1], cf2_pos[2], c= 'k')
    # ax.set_xlabel('Position X')
    # ax.set_ylabel('Position Y')
    # ax.set_zlabel('Position Z')
    # ax.set_title('Drone real-time position')
    ax.axes.set_xlim3d(left= -10, right = 10)
    ax.axes.set_ylim3d(bottom = -10, top = 10)
    ax.axes.set_zlim3d(bottom = 0, top = 3)


if __name__ == "__main__":
    mainwindow = Tkinter.Tk()
    mainwindow.title("CrazyTrain - Realtime simulation control")
    mainwindow.geometry("350x300")

    frame = Tkinter.Frame(mainwindow)  #inside box
    frame.pack()

    traj_print = Tkinter.Checkbutton(frame, text="Show trajectory" )
    traj_print.pack()
    start_print=Tkinter.Button(frame, text = "Start trajectory",  bg='green')
    start_print.pack()
    start_print = Tkinter.Button(frame, text = "Start/Pause visualisation",  bg='green', command = Sim_Paus)
    start_print.pack()

    anim_running = True
    swarm = Crazyswarm()
    allcfs = swarm.allcfs
    log_x = []
    log_y = []
    log_z = []
    for cf in allcfs.crazyflies:
        log_x.append([])
        log_y.append([])
        log_z.append([])

    fig = plt.figure()
    ax = Axes3D(fig)
    ax.set_xlabel('Position X')
    ax.set_ylabel('Position Y')
    ax.set_zlabel('Position Z')
    ax.set_title('Drone real-time position')
    ax.axes.set_xlim3d(left= -10, right = 10)
    ax.axes.set_ylim3d(bottom = -10, top = 10)
    ax.axes.set_zlim3d(bottom = 0, top = 3)

    anim = animation.FuncAnimation(fig, animate_func, interval=100, blit=False)
    plt.show()
