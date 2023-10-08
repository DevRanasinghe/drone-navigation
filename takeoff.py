#!/usr/bin/env python3

import rospy
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from quadrotor_receive.msg import Dronepoints

class Takeoff:

    
    current_state = State()
    pos = Dronepoints()
    
    def state_cb(self,msg):
        global current_state
        self.current_state = msg

    def callback_pose(self,msg):
        global pos
        self.pos = msg
        print(self.pos)
    
    def __init__(self):
        rospy.init_node('takeoff_node', anonymous=True)
        self.set_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        #self.local_pos_pub = rospy.Subscriber('/mavros/local_position/pose')
        self.pose_sub = rospy.Subscriber("poser",Dronepoints, self.callback_pose)
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.rate = rospy.Rate(20) # Hz
    
    # Wait for FCU connection
        while not rospy.is_shutdown() and not self.current_state.connected:
            self.rate.sleep()

    # Set initial position target to current position
        self.pose = PoseStamped()
        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 3

        for i in range(100):
            self.set_pos_pub.publish(self.pose)
            self.rate.sleep()
    
    # Arm the vehicle
        self.arming_client(True)
        while not rospy.is_shutdown() and not self.current_state.armed:
            self.rate.sleep()
    # Set mode to "OFFBOARD"
        self.set_mode_client(custom_mode="OFFBOARD")

    #pose.pose.position.z = 5
    #for i in range(100):
    #    local_pos_pub.publish(pose)
    #    rate.sleep()
    # Wait for mode change and arming
        while not rospy.is_shutdown() and self.current_state.mode != "OFFBOARD":
            self.rate.sleep()
    
        rospy.loginfo("done")


    def main_takeoff(self):
        
        while not rospy.is_shutdown(): #and abs(pose.z - 2) > 0.1:
            self.set_pos_pub.publish(self.pose)
            self.rate.sleep()

        rospy.loginfo("Takeoff successful!")
        

