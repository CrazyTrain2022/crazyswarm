from pycrazyswarm import Crazyswarm
import numpy as np
import uav_trajectory


if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    TRIALS = 2

    for i in range(TRIALS):
        cf1 = swarm.allcfs.crazyflies[0]
        cf2 = swarm.allcfs.crazyflies[1]
        cf3 = swarm.allcfs.crazyflies[2]
        cf4 = swarm.allcfs.crazyflies[3]

        allcfs.takeoff(targetHeight=1.0, duration=2.0)
        timeHelper.sleep(2.5)

        cf1.goTo(np.array([-2,0,0.5]), 0, 2.0)
        cf2.goTo(np.array([-1,0,0.5]), 0, 2.0)
        cf3.goTo(np.array([0,0,0.75]), 0, 2.0)
        cf4.goTo(np.array([1,0,0.9]), 0, 2.0)

        cf1.goTo(np.array([-1.5,0,0.5]),0, 2.0)
        cf2.goTo(np.array([-0.5,0,0.5]),0, 2.0)
        cf3.goTo(np.array([0.25,0,0.5]),0, 2.0)
        cf4.goTo(np.array([1.585,0,0.5]),0, 2.0)

        cf1.goTo(np.array([-1.5,0,2]),0, 2.0)
        cf2.goTo(np.array([-0.5,0,1.5]),0, 2.0)
        cf3.goTo(np.array([0.5,0,0.75]),0, 2.0)
        cf4.goTo(np.array([2.17,0,0.9]),0, 2.0)

        cf1.goTo(np.array([-2,0,2]),0, 2.0)
        cf2.goTo(np.array([-1,0,1.5]),0, 2.0)
        cf3.goTo(np.array([0.25,0,1]),0, 2.0)
        cf4.goTo(np.array([2.17,0,1.5]),0, 2.0)

        cf1.goTo(np.array([-2,0,0.5]),0, 2.0)
        cf2.goTo(np.array([-1,0,0.5]),0, 2.0)
        cf3.goTo(np.array([0,0,0.75]),0, 2.0)
        cf4.goTo(np.array([1.67,0,1.5]),0, 2.0)

        cf4.goTo(np.array([1.67,0,1.1]),0, 2.0)
        cf4.goTo(np.array([1.585,0,0.9]),0, 2.0)
        cf4.goTo(np.array([1.5,0,1.1]),0, 2.0)
        cf4.goTo(np.array([1.5,0,1.5]),0, 2.0)
        cf4.goTo(np.array([1,0,1.5]),0, 2.0)
        cf4.goTo(np.array([1,0,0.9]),0, 2.0)

        timeHelper.sleep(1.0)


        allcfs.land(targetHeight=0.06, duration=2.0)
        timeHelper.sleep(3.0)