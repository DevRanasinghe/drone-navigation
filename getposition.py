#!/usr/bin/env python3

import rospy
from mavros_msgs import PoseStamped

current_state = PoseStamped()

def callback(msg):
    global current_state
    current_state = msg

rospy.init_node('position_node', anonymous=True)
rospy.Subscriber('/mavros/local_position/pose',PoseStamped,callback)    
ros_rate = rospy.Rate(20)

while not rospy.is_shutdown():
    x = current_state.pose.position.x
    y = current_state.pose.position.y
    z = current_state.pose.position.z

    print("x = "+str(x)+" y = "+str(y)+" z = "+str(z))

    ros_rate.sleep()
