#from crazyswarm.ros_ws.src.crazyswarm.scripts.pycrazyswarm.crazyflie import TimeHelper
import rospy
from rosgraph_msgs.msg import Log
import os

import sys

def callback(data):

    if (data.function == "logWarn"):
        if (data.msg[:7] == "Dynamic"): 

            os.system('gnome-terminal -- bash landDrone.sh')
            sys.exit("Lost connection")
  



def listener():

 
    rospy.init_node('emergency111'),# anonymous=True)
    test = 1; 
    rospy.Subscriber('/rosout', Log, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

listener()