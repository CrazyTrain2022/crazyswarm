from pycrazyswarm import Crazyswarm
import numpy as np
import uav_trajectory


if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    cf1 = swarm.allcfs.crazyflies[0]
    cf2 = swarm.allcfs.crazyflies[1]
    cf3 = swarm.allcfs.crazyflies[2]
    cf4 = swarm.allcfs.crazyflies[3]

     # activate enable avoidance
    cfs = swarm.allcfs.crazyflies
    xy_radius = 0.2
    radii = xy_radius * np.array([1.0, 1.0, 3.0])

    for i, cf in enumerate(cfs):
        others = cfs[:i] + cfs[(i+1):]
        cf.enableCollisionAvoidance(others, radii)

    cf1.takeoff(targetHeight = 2.0, duration = 2.0)
    cf2.takeoff(targetHeight = 1.5, duration = 2.0)
    cf3.takeoff(targetHeight = 0.75, duration = 2.0)
    cf4.takeoff(targetHeight = 1.5, duration = 2.0)

    #allcfs.takeoff(targetHeight=0.5, duration=2.0)
    timeHelper.sleep(2.5)


    TRIALS = 2

    for i in range(TRIALS):
        

        

        cf1.goTo(np.array([-2,0,2.0]), 0, 2.0)
        cf2.goTo(np.array([-1,0,1.5]), 0, 2.0)
        cf3.goTo(np.array([0.0, 0, 0.75]), 0, 2.0)
        cf4.goTo(np.array([2.17,0,1.5]), 0, 2.0)

        timeHelper.sleep(3.0)

        cf1.goTo(np.array([-2,0,0.6]),0, 2.0)
        cf2.goTo(np.array([-1.0,0,0.6]),0, 2.0)
        cf3.goTo(np.array([0.0,0.0,0.75]), 0, 2.0)
        timeHelper.sleep(0.5)
        cf3.goTo(np.array([0.25,0.0,0.5]), 0, 2.0)
        timeHelper.sleep(0.5)
        cf3.goTo(np.array([0.5,0.0,0.75]), 0, 2.0)
        timeHelper.sleep(0.5)
        cf3.goTo(np.array([0.25,0.0,1.0]), 0, 2.0)
        timeHelper.sleep(0.5)
        cf3.goTo(np.array([0.0,0.0,0.75]), 0, 2.0)

        cf4.goTo(np.array([1.67,0,1.5]), 0, 2.0)    

        timeHelper.sleep(3.0)

        cf1.goTo(np.array([-1.5,0,0.6]),0, 2.0)
        cf2.goTo(np.array([-0.5,0,0.6]),0, 2.0)

        cf4.goTo(np.array([1.67,0,1.07]), 0, 2.0) 

        timeHelper.sleep(3.0)

        cf1.goTo(np.array([-1.5,0,2.0]),0, 2.0)
        cf2.goTo(np.array([-0.5,0,1.5]),0, 2.0)
        cf4.goTo(np.array([1.585,0,0.9]), 0, 2.0) 
        cf4.goTo(np.array([1.5,0,1.07]), 0, 2.0)  

        timeHelper.sleep(3.0)

        cf1.goTo(np.array([-2.0,0.0,2.0]),0, 2.0)
        cf2.goTo(np.array([-1,0,1.5]),0, 2.0)
        cf3.goTo(np.array([0.25,0.0,0.5]), 0, 2.0)
        timeHelper.sleep(0.5)
        cf3.goTo(np.array([0.5,0.0,0.75]), 0, 2.0)
        timeHelper.sleep(0.5)
        cf3.goTo(np.array([0.25,0.0,1.0]), 0, 2.0)
        timeHelper.sleep(0.5)
        cf3.goTo(np.array([0.0,0.0,0.75]), 0, 2.0)
        cf4.goTo(np.array([1.5,0,1.5]), 0, 2.0) 

        timeHelper.sleep(2.0)
        cf4.goTo(np.array([1,0,1.5]), 0, 2.0) 
        timeHelper.sleep(2.0)
        cf4.goTo(np.array([1,0,0.9]), 0, 2.0)
        timeHelper.sleep(2.0)
        cf4.goTo(np.array([1.585,0,0.5]), 0, 2.0)
        timeHelper.sleep(0.50)
        cf4.goTo(np.array([2.17,0,0.9]), 0, 2.0)
        timeHelper.sleep(2.0)
        cf4.goTo(np.array([2.17,0,1.5]), 0, 2.0)      


        timeHelper.sleep(3.0)


    allcfs.land(targetHeight=0.06, duration=2.0)
    timeHelper.sleep(3.0)