import numpy as np
import sys
import csv

# run using following line:
# python3 yaw_generation.py drone_X_trajectory.csv yaw.csv

def main(drone_traj, yaw, name):
    dt_no_yaw = np.genfromtxt(drone_traj, delimiter=',')
    dt_no_yaw = dt_no_yaw[1:]
    
    yaw = np.genfromtxt(yaw, delimiter=',')
    yaw = yaw[0:]
    yaw = yaw*np.pi/180


    dt_w_yaw = dt_no_yaw

    for i in range(len(yaw)-1):
        dt_w_yaw[i,25] = yaw[i]
        dt_w_yaw[i,26] = (yaw[i+1]-yaw[i])/dt_w_yaw[i,0]

    head = "duration,x^0,x^1,x^2,x^3,x^4,x^5,x^6,x^7,y^0,y^1,y^2,y^3,y^4,y^5,y^6,y^7,z^0,z^1,z^2,z^3,z^4,z^5,z^6,z^7,yaw^0,yaw^1,yaw^2,yaw^3,yaw^4,yaw^5,yaw^6,yaw^7,"
    np.savetxt('crazyswarm/ros_ws/src/crazyswarm/scripts/drone' + name + 'trajectory.csv', dt_w_yaw, delimiter=',', header=head)

if __name__ == '__main__':
    args = sys.argv[1:]
    drone_traj = args[0]
    yaw = args[1]
    name = args[2]

    main(drone_traj,yaw,name)
