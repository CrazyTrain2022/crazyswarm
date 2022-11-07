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
    i = 0
    path = "."
    trajectory_files = [f for f in os.listdir(path) if os.path.isfile(os.path.join(path, f))]
    # print(trajectory_pathfiles)
    for file in trajectory_files:
        if("trajectory.csv" in file):
            traj_lst.append(uav_trajectory.Trajectory())
            traj_lst[i].loadcsv(file)
            i += 1

    
    TRIALS = 1
    TIMESCALE = 1.0
    traj1 = traj_lst[0]
    print(traj_lst)
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
