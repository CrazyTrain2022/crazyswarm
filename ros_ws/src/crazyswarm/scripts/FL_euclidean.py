"""Follow the leader"""
#!/usr/bin/env
from pycrazyswarm import Crazyswarm
import numpy as np
import uav_trajectory
from scipy import optimize
import matplotlib.pyplot as plt


def cost_fun(cf2_goal_pos, cf1_pos, d1, d2):
    return np.sqrt((cf1_pos[0] - cf2_goal_pos[0] + d1)**2 + (cf1_pos[1] - cf2_goal_pos[1] + d2)**2)
    

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    """create trajectory"""
    traj1 = uav_trajectory.Trajectory()
    traj1.loadcsv("figure8.csv") #Choose trajectory for cf1
    TRIALS = 1
    TIMESCALE = 1.0

    for i in range(TRIALS):
        cf1 = swarm.allcfs.crazyflies[0]
        cf2 = swarm.allcfs.crazyflies[1]
        cf1.uploadTrajectory(0, 0, traj1)
        cf2.uploadTrajectory(0, 0, traj1)

        allcfs.takeoff(targetHeight=1.0, duration=2.0)
        timeHelper.sleep(2.5)
        for cf in allcfs.crazyflies:
            pos = np.array(cf.initialPosition) + np.array([0, 0, 1.0])
            cf.goTo(pos, 0, 2.0)
        timeHelper.sleep(2.5)

        

        cf1.startTrajectory(0, timescale=TIMESCALE)
        cf2.startTrajectory(0, timescale=TIMESCALE)
        nbrloops= 10*(traj1.duration * TIMESCALE +2)
        n = 0
        cf1_pose_log = np.empty((0,3),float)
        cf2_goal_log = np.empty((0,3),float)
        while n < nbrloops:
            cf1_pos = cf1.position()
            cf2_pos = cf2.position()

            opti_pos = optimize.minimize(cost_fun, cf2_pos, args = (cf1_pos, 0.5, 0.5))

            cf2_pose_goal = opti_pos.x

            cf2_pose_goal = np.array([opti_pos[0], opti_pos[1], cf1_pos[2] ])

            cf1_pose_log = np.append(cf1_pose_log, cf1_pos, axis=0)
            cf2_goal_log = np.append(cf2_goal_log, cf2_pose_goal, axis=0)

            print("Pose cf1")
            print(cf1_pos)
            print("Optimal Pose cf2")
            print(cf2_pose_goal)

            # cf2.cmdPosition(cf2_pose_goal,0)
            timeHelper.sleep(0.1) #If the script does not work, check this.
            n = n + 1
        

        timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)
        cf2.NotifySetpointsStop()

        allcfs.land(targetHeight=0.06, duration=2.0)
        timeHelper.sleep(3.0)

        #plot cf1 pose and cf2 goal pose.
        plt.plot(cf1_pose_log[:,0],cf1_pose_log[:,1], 'r')
        plt.plot(cf2_goal_log[:,0],cf2_goal_log[:,1], 'b')
        plt.show()

