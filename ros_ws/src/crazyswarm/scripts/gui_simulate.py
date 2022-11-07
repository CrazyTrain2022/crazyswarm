#!/usr/bin/env python

import numpy as np

import tkinter as Tkinter

import os
import glob
import yaml

from threading import Thread,Event

from pycrazyswarm import *
import uav_trajectory

TRIALS = 1
TIMESCALE = 1.0

class Simulate:
    def __init__(self, mainwindow_):
        self.mainwindow = mainwindow_
        self.show_traj = 2


    # writes to plot_trajectories.yaml file for visMatlotlib.py to get instructions to show trajectoies or not
    def write_yaml(self):
        # check which drones are used
        # path = "/home/karl/TSRT10/visionen/GUI/points_csv/"
        path = "../../../../../GUI/points_csv/"
        waypoint_files = [f for f in os.listdir(path) if os.path.isfile(os.path.join(path, f))]
        drones = []
        show_plot_vals = []
        for file in waypoint_files:
            if("waypoints.csv" in file):
                drones.insert(0,file.replace('drone', '').replace('waypoints.csv',''))
                show_plot_vals.append(self.show_traj)

        # make dictionary 
        plot_data = dict([("settings",[])])
        i = 0
        for drone in drones:
            a = show_plot_vals[i]
            plot_data["settings"].append(dict(["i"+str(drone), ("plot", a)]))
            # sett = {"i": str(drone)}
            # sett["plot"] = a
            i += 1

        # write to yaml file plot_trajectories
        with open("plot_trajectories.yaml", 'w') as outfile:
            yaml.dump(plot_data, outfile, default_flow_style=False)

    # update bool for yaml file
    def update_yaml(self, bool_show_traj):
        if bool_show_traj.get() == 1:
            self.show_traj = 2
            self.write_yaml()
        elif bool_show_traj.get() == 0:
            self.show_traj = 1
            self.write_yaml()

    # start trajectory
    def traj_func(self):

        swarm = Crazyswarm()
        self.timeHelper = swarm.timeHelper
        self.allcfs = swarm.allcfs

        # activate enable avoidance
        cfs = swarm.allcfs.crazyflies
        xy_radius = 0.2
        radii = xy_radius * np.array([1.0, 1.0, 3.0])

        for i, cf in enumerate(cfs):
            others = cfs[:i] + cfs[(i+1):]
            cf.enableCollisionAvoidance(others, radii)

        traj_lst = []
        u = 0
        for cf in self.allcfs.crazyflies:
            traj_lst.append(uav_trajectory.Trajectory())
            traj_lst[u].loadcsv("drone" + str(u+1) + "trajectory.csv")
            cf.uploadTrajectory(0, 0, traj_lst[u])
            #trajs[u].loadcsv("rob_yaw_traj.csv")
            u += 1

        TIMESCALE = 1.0
        
        j = 0
        for cf in self.allcfs.crazyflies:
            pos = traj_lst[j].eval(0).pos
            z = pos[2]
            j += 1
            cf.takeoff(targetHeight=z, duration=2.0)
        self.timeHelper.sleep(2.5)

        self.allcfs.startTrajectory(0, timescale=TIMESCALE)
        self.timeHelper.sleep(traj_lst[0].duration * TIMESCALE + 2.0)

        self.land_func()

    # landing drone, both used at end of flight and at emergency
    def land_func(self):
        # land drone
        self.allcfs.land(targetHeight=0.02, duration=2.0)
        self.timeHelper.sleep(3.0)

        self.close_window()

    def emergency_stop(self):
        print("Emergency stop")
        self.land_func()

    def remove_csv_files(self):
        # remove drone*trajectory.csv files
        files = glob.glob('*')
        for f in files:
            if("drone" in f and "trajectory.csv" in f):   # remove everything not being a folder
                os.remove(f)

    def close_window(self):
        self.remove_csv_files()
        self.mainwindow.destroy()

def key_pressed(event, sim_obj):
        if(event.keysym == "space"):
            sim_obj.emergency_stop()


if __name__ == "__main__":
    mainwindow = Tkinter.Tk()
    mainwindow.title("CrazyCrowd - Simulation control")
    mainwindow.geometry("350x300")

    frame = Tkinter.Frame(mainwindow)  #inside box
    frame.pack()

    sim_obj = Simulate(mainwindow)
    bool_show_traj=Tkinter.BooleanVar(frame, value=1)
    sim_obj.update_yaml(bool_show_traj)
    
    traj_print = Tkinter.Checkbutton(frame, text="Show trajectory", variable=bool_show_traj, command = lambda: sim_obj.update_yaml(bool_show_traj), )
    traj_print.pack()
    start_print=Tkinter.Button(frame, text = "Start trajectory",  bg='green', command = sim_obj.traj_func)
    start_print.pack()
    land_print=Tkinter.Button(frame, text = "Emergency", bg='red', width=50, height=50, command = sim_obj.emergency_stop)
    land_print.pack()
    
    mainwindow.bind("<KeyPress>", lambda event: key_pressed(event, sim_obj))

    mainwindow.protocol("WM_DELETE_WINDOW", sim_obj.close_window)
    mainwindow.mainloop()
