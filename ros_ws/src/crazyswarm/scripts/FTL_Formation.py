"""Follow the leader Formation"""
#!/usr/bin/env
from pycrazyswarm import Crazyswarm
import numpy as np
import uav_trajectory
import math
import os

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    """create trajectory"""
   # read all trajectory files and load into trajectory objects
    traj_lst = []
    traj_lst_string = []
    for i in range(4):
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
    for k in traj_lst_string:
        print(k)
    i = 0
    for j in traj_lst_string:
        if(len(j) > 1):
            traj_lst.append(uav_trajectory.Trajectory())
            traj_lst[i].loadcsv(j)
            print("appending :", j, i)
            i += 1
   
   
   
    traj1 = traj_lst[0] #Choose trajectory for the Leader
    TRIALS = 1
    TIMESCALE = 1.0

    for i in range(TRIALS):
        cf1 = swarm.allcfs.crazyflies[0]
        
        

        cf1.uploadTrajectory(0, 0, traj1)
        
        xy_radius = 0.2
        radii = xy_radius * np.array([1.0, 1.0, 3.0])

        for i, cf in enumerate(swarm.allcfs.crazyflies):
            others = swarm.allcfs.crazyflies[:i] + swarm.allcfs.crazyflies[(i+1):]
            cf.enableCollisionAvoidance(others, radii)
        

        allcfs.takeoff(targetHeight=1.0, duration=2.0)
        timeHelper.sleep(2.5)
        for cf in allcfs.crazyflies:
            pos = np.array(cf.initialPosition) + np.array([0, 0, 1.0])
            cf.goTo(pos, 0, 2.0)
        timeHelper.sleep(2.5)

        

        cf1.startTrajectory(0, timescale=TIMESCALE)
        nbrloops= 10*(traj1.duration * TIMESCALE +2)
        n = 0
        safe_dis = 0.4
        while n < nbrloops:
            i = 0
            for cf in allcfs.crazyflies:
                
                if(i == 0):
                    cf_Leader_pos = cf.position()
                    
                else:
                    cf_follow_pos = cf.position()
                    Dis = math.dist(cf_Leader_pos, cf_follow_pos)

                    if (Dis > safe_dis):
                        cf.cmdPosition(cf_Leader_pos, 0)
                    else:
                        cf_safe_dis = np.array([cf_follow_pos[0], cf_follow_pos[1], cf_Leader_pos[2]]) 
                        cf.cmdPosition(cf_safe_dis,0)
                
                i += 1

            
            timeHelper.sleep(0.1) #If the script does not work, check this.
            n = n + 1
        

        timeHelper.sleep(1.0)

        # pos_offset = cf1pos + offset
        # problem: för många pos per sekund. 
        # cf2 tar pos på cf1 som goTo(pos_offset,0,2)
        
        #rospy.init_node('test_high_level')
        #cf = crazyflie.Crazyflie("crazyflie2", "/vicon/crazyflie1/crazyflie1")
        
        #allcfs.NotifySetpointsStop()
        
        # cf1.setParam("commander/enHighLevel", 1)
        # cf2.setParam("commander/enHighLevel", 1)
        # cf3.setParam("commander/enHighLevel", 1)
        # cf4.setParam("commander/enHighLevel", 1)

        allcfs.land(targetHeight=0.06, duration=2.0)
        timeHelper.sleep(3.0)


    for file in trajectory_files:
        if("trajectory.csv" in file):
            os.remove(file)
            