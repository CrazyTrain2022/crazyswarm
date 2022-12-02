"""Follow the leader Line"""
#!/usr/bin/env
from pycrazyswarm import Crazyswarm
import numpy as np
import uav_trajectory
import math
import os

# Definitation of the function that calulate the distance between two cfs (crazyflies)
# and if the actual distance is larger than the safe dist. the follower can flights, 
# else it stay at same posistion.

def pos_calc(cf, pos0, pos1, safe, leaderz):
    Dis = math.dist(pos0, pos1)

    if (Dis > safe):
        cf_safe_dis = np.array([pos0[0], pos0[1], leaderz])
        cf.cmdPosition(cf_safe_dis, 0)
    else:
        cf_safe_dis = np.array([pos1[0], pos1[1], leaderz]) 
        cf.cmdPosition(cf_safe_dis,0)





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
        cf2 = swarm.allcfs.crazyflies[1]
        cf3 = swarm.allcfs.crazyflies[2]
        cf4 = swarm.allcfs.crazyflies[3]

        cf1.uploadTrajectory(0, 0, traj1)
        
        xy_radius = 0.2
        radii = xy_radius * np.array([1.0, 1.0, 3.0])

        for i, cf in enumerate(swarm.allcfs.crazyflies):
            others = swarm.allcfs.crazyflies[:i] + swarm.allcfs.crazyflies[(i+1):]
            cf.enableCollisionAvoidance(others, radii)
        

        # Take off all the drones that are in the allfcs list to 1m above the ground during 2s.
        # then sleep to lets the drones are in diserted hight.
        allcfs.takeoff(targetHeight=1.0, duration=2.0)
        timeHelper.sleep(2.5) 

        # Here the drones fly to the 1m high.
        pos = np.array(cf1.initialPosition) + np.array([0, 0, 1])
        cf1.goTo(pos, 0, 2.0)
        timeHelper.sleep(2.0)
        cf2.goTo(pos - np.array([0.3, 0, 0]), 0, 2.0)
        timeHelper.sleep(2.0)
        cf3.goTo(pos - np.array([0.6, 0, 0]), 0, 2.0)
        timeHelper.sleep(2.0)
        cf4.goTo(pos - np.array([0.9, 0, 0]), 0, 2.0)
        timeHelper.sleep(2.5)

        cf1.startTrajectory(0, timescale=TIMESCALE)


        # In this loop the drone with lowest id-number take the leader roll. 
        # other cfs fly if the disatnce between them are larger then safe distance,
        # else hover at same position as before.

        nbrloops= 10*(traj1.duration * TIMESCALE +2) # 10Hz
        n = 0
        m = 0
        safe_dis = 0.2
        while n < nbrloops:
            i = 0
            
            for cf in allcfs.crazyflies:
                
                if(i == 0):
                    cf_Leader_pos = cf.position()
                    z = cf_Leader_pos[2]
                elif(i == 1):
                    cf_follow1_pos = cf.position()
                    pos_calc(cf, cf_Leader_pos, cf_follow1_pos, safe_dis, z)
                
                elif(i == 2):

                    if (m == 0):
                        timeHelper.sleep(1.0)
                        m = 1
                        cf_follow2_pos = cf.position()
                        pos_calc(cf, cf_follow1_pos,cf_follow2_pos, safe_dis, z)

                    else:
                        cf_follow2_pos = cf.position()
                        pos_calc(cf, cf_follow1_pos,cf_follow2_pos, safe_dis, z) 
                
                else:
                    if(m == 1):
                        timeHelper.sleep(3.0)
                        m = 2
                    
                        cf_follow3_pos = cf.position()
                        pos_calc(cf, cf_follow2_pos,cf_follow3_pos, safe_dis, z)
                    else:
                        cf_follow3_pos = cf.position()
                        pos_calc(cf, cf_follow2_pos,cf_follow3_pos, safe_dis, z)

                i += 1

            timeHelper.sleep(0.1) #If the script does not work, check this.
            n = n + 1
        

        timeHelper.sleep(1.0)


        allcfs.land(targetHeight=0.06, duration=2.0)
        timeHelper.sleep(3.0)

    for file in trajectory_files:
        if("trajectory.csv" in file):
            os.remove(file)
