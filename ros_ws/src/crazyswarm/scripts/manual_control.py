from pycrazyswarm import genericJoystick
from pycrazyswarm import Crazyswarm
import numpy as np
import math
import os
import sys

import time


PROJECT_PATH = "/home/crazycrowd/crazycrowd/visionen/"  #File path to the project folder 'visionen'
MODES = np.array(["normal", "yaw_independant"])         #Manual control modes

def main(flags):
    mode_index = 0
    mode = MODES[mode_index]    #Initialize first manual control mode to "normal"
    swarm = Crazyswarm(flags=flags)
    timeHelper = swarm.timeHelper

    #The drone selected for manual control -  should input from GUI to here
    #Currently only selects the first drone in the swarm list of crazyflies
    selected_drone = 0

    #Only select the drone for manual control given by the index 'selected_drone'
    cf = swarm.allcfs.crazyflies[selected_drone]
    #Run the xbox driver (xboxdrv --silent) from pydatertc.sh in a new terminal
    command = 'sudo '+ PROJECT_PATH + 'pydatertc.sh'
    os.system("gnome-terminal -e 'bash -c \""+command+";bash\"'")

    #Let system catch the connected xbox controller by short sleep
    time.sleep(0.5)
    xbox = genericJoystick.Joystick(timeHelper)
    
    #Initial position of the drone
    x = 0
    y = 0
    z = 0
    yaw = 0
    move_arr = np.empty([1,6]) #Array for containing current movement inputs
    wait_for_release = True #Used for reading button presses only once at a time (for mode switches)

    #Read input from xbox-controller
    while(xbox.checkIfButtonIsPressed()==False):
        #Input: [[ls_lr,ls_ud,rs_lr,rs_ud,RT,LT,dp_lr,dp_ud],[A,B,X,Y,LB,RB,BACK,START,GUIDE,ls_p,rs_p]]
        #ls - left stick
        #rs - right stick
        #lr - right/left
        #ud - up/down
        #dp - D-pad ("arrows"-multibutton on the Xbox360 controller)
        #p - pushed
        input = xbox.js.read(xbox.joyID)

        landing = input[1][1]   #Land when pressing "B" on xbox-controller
        change_mode = input[1][3]   #Change manual control mode when pressing "Y" on xbox-controller

        if landing == 1:
            cf.land(targetHeight = 0.06, duration = 2.0)
            timeHelper.sleep(3)

        elif change_mode == 1 and wait_for_release == True:
            mode_index = abs(mode_index-1)
            mode = MODES[mode_index]
            print("Manual control mode: "+mode)
            wait_for_release = False

        elif change_mode == 0 and wait_for_release == False:
            wait_for_release = True

        else:
            #Only select, from input, the values used for movement and ignore values close to zero
            #such that it is not sensitive to unwanted user input
            for i in range(0,len(move_arr[0]),1):
                val = input[0][i]
                if -0.2 < val < 0.2:
                    move_arr[0][i] = 0
                else:
                    move_arr[0][i] = val

            yaw_factor = 2*math.pi/20   #Factor the sensitivity to yaw input
            factor = 0.1                #Factor the sensitivity to x,y,z-movement input

            #Stick inputs
            ls_lr = move_arr[0][0]    #Left stick, left/right
            ls_ud = move_arr[0][1]    #Left stick, up/down
            rs_lr = move_arr[0][2]    #Right stick, left/right
            rs_ud = move_arr[0][3]    #Right stick, up/down

            #Calculate factored outputs & fix sign conflicts between controller
            #inputs and the global coordinate system
            x_out = ls_lr*factor
            y_out = -ls_ud*factor
            z_out = -rs_ud*factor
            yaw_out = rs_lr*yaw_factor

            #Update position and execute movement
            pos = np.array(cf.initialPosition) + np.array([x, y, z])
            cf.cmdPosition(pos, yaw)
            timeHelper.sleep(0.1)

            #Normal mode: forward is always in the heading, yaw angle, of the drone
            if mode == "normal":
                yaw += yaw_out #rs_lr
                x += x_out*math.cos(-yaw) - y_out*math.sin(-yaw) #ls_lr (assuming x axis left/right)
                y += x_out*math.sin(-yaw) + y_out*math.cos(-yaw) #ls_ud (assuming y axis forward/backward)
                z += z_out #rs_ud (z axis up/down)
            
            #yaw_independent mode: movement is always in regards to the global coordinate system
            #regardless of the drone heading
            elif mode == "yaw_independant":
                #Note that these do not consider angle
                x += x_out #ls_lr (assuming x axis left/right)
                y += y_out #ls_ud (assuming y axis forward/backward)
                z += z_out #rs_ud (z axis up/down)
                yaw += yaw_out #rs_lr
            else:
                print("Failed to enter a manual control mode")



if __name__ == "__main__":
    # input flags
    flags = sys.argv[2:]
    main(flags[0])


