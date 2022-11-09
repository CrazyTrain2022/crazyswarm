from test_subscriber import *
from pycrazyswarm import *
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

if __name__ == '__main__':
    swarm = Crazyswarm()
    allcfs = swarm.allcfs

    fig = plt.figure(1)
    ax = Axes3D(fig)
    ax.set_xlabel('Position X')
    ax.set_ylabel('Position Y')
    ax.set_zlabel('Position Z')
    ax.set_title('Drone real-time position')
    ax.axes.set_xlim3d(left= -10, right = 10)
    ax.axes.set_ylim3d(bottom = -10, top = 10)
    ax.axes.set_zlim3d(bottom = 0, top = 3)
    Position_log = np.array([])

    time_after_loop = time.process_time()
    freq = 0.1
    k = 0
    cf_list = []
    for cf in allcfs.crazyflies:
            cf_list.append(cf)
            
            
    while k<100:
        # pos_list = np.zeros(3,4)
        time_before_loop = time.process_time()
        print("Time: ", time_before_loop)
        
        print("cf: ", cf_list)
        cf1_pos = cf_list[0].position() 
        #cf2_pos = cf_list[1].position() 
        print(cf1_pos)
        ax.scatter(cf1_pos[0], cf1_pos[1], cf1_pos[2], c= 'b')
        #ax.scatter(cf2_pos[0], cf2_pos[1], cf2_pos[2], c= 'k')
        plt.pause(0.05)
            # i += 1
                
            # Position_log[i] = pos_list
        time.sleep(0.05)
        time_after_loop = time.process_time()
        print(time_after_loop)
        k+=1
    plt.show()
    
   




