#!/usr/bin/env python

import numpy as np
import os

from pycrazyswarm import *
import uav_trajectory

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs


    # activate enable avoidance
    cfs = swarm.allcfs.crazyflies
    xy_radius = 0.2
    radii = xy_radius * np.array([1.0, 1.0, 3.0])

    for i, cf in enumerate(cfs):
        others = cfs[:i] + cfs[(i+1):]
        cf.enableCollisionAvoidance(others, radii)


    # read all trajectory files and load into trajectory objects
    traj_lst = []
    traj_lst_string = []
    for i in range(3):
        traj_lst_string.append(str(i+1))
        
    path = "."
    trajectory_files = [f for f in os.listdir(path) if os.path.isfile(os.path.join(path, f))]
    #put the right trajectory file on the right place
    for file in trajectory_files:
        if("trajectory.csv" in file):
            if "1" in file:
                traj_lst_string[0] = file
            elif "2" in file:
                traj_lst_string[1] = file
            elif "3" in file:
                traj_lst_string[2] = file
            else:
                traj_lst_string[3] = file
    #Check if actual trajectory file and put them in trajectory list so they come in the right order
    i = 0
    for j in traj_lst_string:
        if(len(j) > 1):
            print("trajectory: ", j)
            traj_lst.append(uav_trajectory.Trajectory())
            traj_lst[i].loadcsv(j)
            i += 1
    TRIALS = 1
    TIMESCALE = 1.0
    traj1 = traj_lst[0]
    print("number of trajectories: ", len(traj_lst))
    print("number of crazyflies: ", len(allcfs.crazyflies))
    j = 0
    for i in range(TRIALS):
        for cf in allcfs.crazyflies:
            cf.uploadTrajectory(0, 0, traj_lst[j])
            j += 1

        allcfs.takeoff(targetHeight=1.0, duration=2.0)
        timeHelper.sleep(2.5)
        for cf in allcfs.crazyflies:
            pos = np.array(cf.initialPosition) + np.array([0, 0, 1.0])
            cf.goTo(pos, 0, 2.0)
        timeHelper.sleep(2.5)
        allcfs.startTrajectory(0, timescale=TIMESCALE)
        timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)
        # allcfs.startTrajectory(0, timescale=TIMESCALE, reverse=True)
        # timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)

        allcfs.land(targetHeight=0.06, duration=2.0)
        timeHelper.sleep(3.0)

    #To delete all the trajectory files when we are done
    for file in trajectory_files:
        if("trajectory.csv" in file):
            os.remove(file)