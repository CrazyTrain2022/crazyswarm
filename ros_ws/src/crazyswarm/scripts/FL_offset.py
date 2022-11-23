"""Follow the leader"""
#!/usr/bin/env
from pycrazyswarm import Crazyswarm
import numpy as np
import uav_trajectory


if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    timeHelper.sleep(5)
            # activate enable avoidance
    cfs = swarm.allcfs.crazyflies
    timeHelper.sleep(5)
    xy_radius = 0.4
    radii = xy_radius * np.array([1.0, 1.0, 3.0])

    for i, cf in enumerate(cfs):
        others = cfs[:i] + cfs[(i+1):]
        cf.enableCollisionAvoidance(others, radii)

    """create trajectory"""
    traj1 = uav_trajectory.Trajectory()
    traj1.loadcsv("custom_test.csv") #Choose trajectory for cf1
    TRIALS = 1
    TIMESCALE = 1.0

    for i in range(TRIALS):
        cf1 = swarm.allcfs.crazyflies[0]
        cf2 = swarm.allcfs.crazyflies[1]
        cf1.uploadTrajectory(0, 0, traj1)

        


        allcfs.takeoff(targetHeight=1.0, duration=2.0)
        timeHelper.sleep(2.5)
        for cf in allcfs.crazyflies:
            pos = np.array(cf.initialPosition) + np.array([0, 0, 1.0])
            cf.goTo(pos, 0, 2.0)
        timeHelper.sleep(2.5)

        

        cf1.startTrajectory(0, timescale=TIMESCALE)
        nbrloops= 10*(traj1.duration * TIMESCALE +2)
        n = 0
        while n < nbrloops:
            cf1_pos = cf1.position()
            cf2_pose_goal = np.array([ cf1_pos[0]-1, cf1_pos[1], cf1_pos[2] ])
            cf2.cmdPosition(cf2_pose_goal,0)
            timeHelper.sleep(0.05) #If the script does not work, check this.
            n = n + 1
        cf2.notifySetpointsStop
        #cf2.setParam("commander/enHighLevel", 1)
        #cf1.setParam("commander/enHighLevel", 1)   
        pos_end = np.array(cf2.initialPosition)
        cf2.goto(pos_end)                          #new

        timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)

        # pos_offset = cf1pos + offset
        # problem: för många pos per sekund. 
        # cf2 tar pos på cf1 som goTo(pos_offset,0,2)


        allcfs.land(targetHeight=0.06, duration=2.0)
        timeHelper.sleep(3.0)
# TODO
# implementera 
#Förfina programmet 
#Euklidisk distans 
#Lägg till funktion för egna missions (inte bara fig. 8) samt antal drönare 
#Kolla möjlighet med att köra med kontroller

