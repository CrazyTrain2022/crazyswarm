"""Follow the leader"""
#!/usr/bin/env
from pycrazyswarm import Crazyswarm
import numpy as np
import uav_trajectory


if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    """create trajectory"""
    traj1 = uav_trajectory.Trajectory()
    traj1.loadcsv("custom_test.csv") #Choose trajectory for cf1
    TRIALS = 1
    TIMESCALE = 1.0

    for i in range(TRIALS):
        cf1 = swarm.allcfs.crazyflies[0]
        cf2 = swarm.allcfs.crazyflies[1]
        cf3 = swarm.allcfs.crazyflies[2]
        cf4 = swarm.allcfs.crazyflies[3]
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
            cf2_pose_goal = np.array([ cf1_pos[0]-0.5, cf1_pos[1], cf1_pos[2] ])
            cf2.cmdPosition(cf2_pose_goal,0)

            cf3_pose_goal = np.array([ cf1_pos[0]-1, cf1_pos[1], cf1_pos[2] ])
            cf3.cmdPosition(cf2_pose_goal,0)

            cf4_pose_goal = np.array([ cf1_pos[0]-1.5, cf1_pos[1], cf1_pos[2] ])
            cf4.cmdPosition(cf2_pose_goal,0)
            
            timeHelper.sleep(0.1) #If the script does not work, check this.
            n = n + 1
        

        timeHelper.sleep(traj1.duration * TIMESCALE + 10.0)

        # pos_offset = cf1pos + offset
        # problem: för många pos per sekund. 
        # cf2 tar pos på cf1 som goTo(pos_offset,0,2)
        
        #rospy.init_node('test_high_level')
        #cf = crazyflie.Crazyflie("crazyflie2", "/vicon/crazyflie1/crazyflie1")
        cf1.setParam("commander/enHighLevel", 1)
        cf2.setParam("commander/enHighLevel", 1)
        cf3.setParam("commander/enHighLevel", 1)
        cf4.setParam("commander/enHighLevel", 1)

        allcfs.land(targetHeight=0.06, duration=2.0)
        timeHelper.sleep(3.0)


