#!/usr/bin/env python3
import numpy as np
import cv2
import sys
from cv2 import aruco
import argparse
import time

import rospy
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped

from std_msgs.msg import _Float64

import takeoff
import call

cb = call.Callback()
to = takeoff.Takeoff()

#msg = 0
#def callback(data):
#    global msg
#    msg = data
 
#rospy.init_node('recieve_vec')

while not rospy.is_shutdown(): 
    
    #if cb.published == True:
    #print("yo")
    #sub = rospy.Subscriber('talker_vec', _Float64, callback)
    #print(msg)
    #msg = float(msg)
    #print(msg)
    to.main_takeoff()
    rospy.Rate(20)
#if __name__ == '__main__': 
#    try:
#        print("good")
#        print(cb.trans)
#        to.main_takeoff(3) 
#    except rospy.ROSInterruptException:
#        pass