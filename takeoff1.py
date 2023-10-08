#!/usr/bin/env python3

import rospy
from mavros_msgs.srv import CommandBool, SetMode, SetModeRequest
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import numpy as np
from quadrotor_receive.msg import Dronepoints
from std_msgs.msg import String
from frames import Frame


class Takeoff:
    
    #current_state = State()
    #vec = 0
    #t_msg = " "
    def __init__(self):

        self.current_state = State()
        self.vec = 0
        self.t_msg_x = 0
        self.t_msg_y = 0
        self.t_msg_z = 0

        self.c_msg_x = 0
        self.c_msg_y = 0
        self.c_msg_z = 0

        self.r_msg_roll = 0
        self.r_msg_pitch = 0
        self.r_msg_yaw = 0

        self.i_msg_roll = 0
        self.i_msg_pitch = 0
        self.i_msg_yaw = 0
        self.i_msg_sca = 0

        self.qr = [0]*4

        self.s_pose = PoseStamped()
        self.v_pose = PoseStamped()
        self.c_pose = PoseStamped()

        self.vel_cmd = Twist()
        self.fp = Frame()
        
        rospy.init_node('takeoff_node', anonymous=True)
        self.setpoint_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.vision_pos_pub = rospy.Publisher('/mavros/vision_pose/pose',PoseStamped,queue_size=10)
        self.vel_setpoints_pos_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel_unstamped',Twist,queue_size=10)
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.imu_pos_sub = rospy.Subscriber('mavros/imu/data',Imu, self.imu_callback)        
        self.p_data_sub = rospy.Subscriber('/poser',Dronepoints, self.pos_callback)
        self.cur_pos_sub = rospy.Subscriber('/mavros/local_position/pose',PoseStamped, self.cur_pos_callback)
        #self.r_data_sub = rospy.Subscriber('talker_rvec',Rotation, self.rot_callback)

        self.rate = rospy.Rate(20) # Hz

    #RAW setpoints and yaw
        #self.r_pose = PoseStamped()
        #self.r_pose.type_mask = int('0000101111111000',2)
        #self.r_pose.coordinate_frame = 1
        #self.r_pose.pose.position = Point(0.0,0.0,-1.0)
        #self.r_pose.velocity = Vector3()
        #self.r_pose.yaw = 0.0
        #self.r_pose.acceleration_or_force = Vector3()
        #self.r_pose.yaw_rate = 0.0
        self.time = rospy.Time.now()
        self.s_pose.header.frame_id = "8"
        #self.s_pose.type_mask = PoseStamped.IGNORE_VX+PoseStamped.IGNORE_VY+PoseStamped.IGNORE_VZ+PoseStamped.IGNORE_AFX+PoseStamped.IGNORE_AFY+PoseStamped.IGNORE_AFZ+PoseStamped.IGNORE_YAW_RATE

        self.s_pose.pose.position.x = 0.0
        self.s_pose.pose.position.y = 1.0
        self.s_pose.pose.position.z = 0.0
       
        self.s_pose.pose.orientation.x = 0.0
        self.s_pose.pose.orientation.y = 0.0
        self.s_pose.pose.orientation.z = 0.0
        self.s_pose.pose.orientation.w = 1.0

        while not rospy.is_shutdown() and not self.current_state.connected:
            print("no connection")
            if self.current_state.connected:
                print("connected")
                break

        #while not rospy.is_shutdown() and (rospy.Time.now() - self.time) < rospy.Duration(2.0):
        #    continue
        #    self.rate.sleep()

        self.s_pose.header.stamp = rospy.Time.now()
        self.v_pose.header.stamp = rospy.Time.now()
       

    def euler_to_quartenion(self,roll,pitch,yaw):
        '''this equation converts Euler angles to quaternions considering "ZYX" sequence where Z is yaw, 
        Y is pitch and X is roll'''

        self.q = [0]*4
        self.q[0] =  np.cos(roll/2)*np.cos(pitch/2)*np.cos(yaw/2) + np.sin(roll/2)*np.sin(pitch/2)*np.sin(yaw/2) #w   
        self.q[1] =  np.sin(roll/2)*np.cos(pitch/2)*np.cos(yaw/2) - np.cos(roll/2)*np.sin(pitch/2)*np.sin(yaw/2)#x   
        self.q[2] =  np.cos(roll/2)*np.sin(pitch/2)*np.cos(yaw/2) + np.sin(roll/2)*np.cos(pitch/2)*np.sin(yaw/2) #y
        self.q[3] =  np.cos(roll/2)*np.cos(pitch/2)*np.sin(yaw/2) - np.sin(roll/2)*np.sin(pitch/2)*np.cos(yaw/2) #z    
        return self.q


    def state_cb(self,msg):
        #global current_state
        self.current_state = msg
        #print(self.current_state)


    def pos_callback(self,data):
        #global t_msg
        self.t_msg_x = data.x 
        self.t_msg_y = data.y
        self.t_msg_z = data.z*-1

        self.r_msg_roll = data.roll
        self.r_msg_pitch = data.pitch
        self.r_msg_yaw = data.yaw

        self.qr = self.euler_to_quartenion(self.r_msg_roll,self.r_msg_pitch,self.r_msg_yaw)


    def imu_callback(self,data):
       
        self.i_msg_roll = data.orientation.x
        self.i_msg_pitch = data.orientation.y
        self.i_msg_yaw = data.orientation.z 
        self.i_msg_sca = data.orientation.w

        #self.qr = [self.i_msg_sca,self.i_msg_roll, self.i_msg_pitch, self.i_msg_yaw]
        #print(data)

    def cur_pos_callback(self,data):

        self.c_msg_x = data.pose.position.x
        self.c_msg_y = data.pose.position.y
        self.c_msg_z = data.pose.position.z 

        print(self.c_msg_y)
        

    def offboard_mode(self):

        for i in range(10):
            
            self.v_pose.pose.position.x = self.t_msg_x
            self.v_pose.pose.position.y = self.t_msg_y
            self.v_pose.pose.position.z = self.t_msg_z 
            
            self.v_pose.pose.orientation.x = self.qr[1]
            self.v_pose.pose.orientation.y = self.qr[2]
            self.v_pose.pose.orientation.z = self.qr[3]
            self.v_pose.pose.orientation.w = self.qr[0]

            self.s_pose.pose.position.x = self.t_msg_x
            self.s_pose.pose.position.y = self.t_msg_y
            self.s_pose.pose.position.z = self.t_msg_z
          
            self.s_pose.pose.orientation.x = self.qr[1]
            self.s_pose.pose.orientation.y = self.qr[2]
            self.s_pose.pose.orientation.z = self.qr[3]
            self.s_pose.pose.orientation.w = self.qr[0]

            self.setpoint_pos_pub.publish(self.s_pose)
            self.vision_pos_pub.publish(self.v_pose)

            self.rate.sleep()

        self.time = rospy.Time.now()

        #--sim
        self.arming_client(True) 
        #--sim
        self.set_mode_client(0,'OFFBOARD')
        
    
    def main_takeoff(self):
        
        self.s_pose.header.stamp = rospy.Time.now()
        self.v_pose.header.stamp = rospy.Time.now()
    # Set initial pose.position target to current pose.position

        #Vision pose.position and pose.orientation
        self.v_pose.pose.position.x = self.t_msg_x
        self.v_pose.pose.position.y = self.t_msg_y
        self.v_pose.pose.position.z = self.t_msg_z

        self.v_pose.pose.orientation.x = self.qr[1]
        self.v_pose.pose.orientation.y = self.qr[2]
        self.v_pose.pose.orientation.z = self.qr[3]
        self.v_pose.pose.orientation.w = self.qr[0]

        #Setpoint pose.position and yaw
        self.s_pose.pose.position.x = 0
        self.s_pose.pose.position.y = self.t_msg_y
        self.s_pose.pose.position.z = self.t_msg_z
        
        self.s_pose.pose.orientation.x = self.qr[1]
        self.s_pose.pose.orientation.y = self.qr[2]
        self.s_pose.pose.orientation.z = self.qr[3]
        self.s_pose.pose.orientation.w = self.qr[0]

        #Setpoint pose.velocity and yaw rate
        #--sim
        x_vel,y_vel,z_vel,yaw_rate = self.fp.velocity((self.c_msg_x-self.t_msg_x),(self.c_msg_y-self.t_msg_y),(self.c_msg_z),self.r_msg_yaw,self.r_msg_roll,(self.c_msg_x-self.t_msg_x),(self.c_msg_y-self.t_msg_y),0.6,0.0)
        #--sim
        print(f"xvel {x_vel}")
        print(f"yvel {y_vel}")
        print(f"zvel {z_vel}")

        self.vel_cmd.linear.x = x_vel
        self.vel_cmd.linear.y = y_vel
        self.vel_cmd.linear.z = z_vel
        self.vel_cmd.angular.x = 0
        self.vel_cmd.angular.y = 0
        self.vel_cmd.angular.z = yaw_rate
        
        #self.setpoint_pos_pub.publish(self.s_pose)
        self.vision_pos_pub.publish(self.v_pose)
        self.vel_setpoints_pos_pub.publish(self.vel_cmd)


if __name__ == '__main__':

    take = Takeoff()
    take.offboard_mode()
    while not rospy.is_shutdown():
        take.main_takeoff()
        take.rate.sleep()