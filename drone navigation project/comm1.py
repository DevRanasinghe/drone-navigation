#!/usr/bin/env python3

import rospy
from std_msgs.msg import String


rospy.init_node('com1', anonymous=True)
talker_pub = rospy.Publisher('talker',String,queue_size=10)
ros_rate = rospy.Rate(20)

while not rospy.is_shutdown():

    hello = "Hello Devin, can you hear me?"
    talker_pub.publish(hello)
    ros_rate.sleep()

    
