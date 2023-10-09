#!/usr/bin/env python3

import rospy
import cv2
import argparse
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Imu
from cv_bridge import CvBridge
from cv2 import aruco
import sys
import os 
from quadrotor_receive.msg import Dronepoints
from frames import Frame


import matplotlib.pyplot as plt
import time
from scipy import signal



bridge = CvBridge()
#fourcc = cv2.VideoWriter_fourcc(*'XVID')  # You can change the codec as needed
#out = cv2.VideoWriter('output.avi', fourcc, 20.0, (424, 240))  # Filename, codec, frames per second, frame size

class Comm2:

    def __init__(self):
     
        rospy.init_node('com2',anonymous=True)
        self.pub_pose = rospy.Publisher('poser',Dronepoints,queue_size=10)
        #self.listener_sub = rospy.Subscriber("imager",CompressedImage,self.com_callback,queue_size=1)
        #--sim
        self.listener_sub = rospy.Subscriber('/image_raw/compressed',CompressedImage,self.com_callback,queue_size=1)
        #--sim
        self.imu_pos_sub = rospy.Subscriber('mavros/imu/data',Imu, self.imu_callback)
        self.ros_rate = rospy.Rate(20)
        
        self.i_msg_roll = 0
        self.i_msg_pitch = 0
        self.i_msg_yaw = 0

        self.cam_yaw_points = 2
        self.cam_yaw = np.array([])
        self.gradient_diff = 0.1
        self.yaw_minimized = 0.3

        self.EWMAy = 0.0
        self.pEWMAy = 0.0
        self.buffY = np.array([])
        self.buffP = np.array([])
        self.buf_len = 10

        self.start_time = 0.00000000
        self.end_time   = 0.00000000
        self.min_value = 1000 
        self.max_value = 0

        self.buffx = np.array([])
        self.buffy = np.array([])
        self.yawd = 0


        self.buffYw = np.array([])
        self.buffYb = np.array([])
        fra = Frame()

        self.buffYw = np.append(self.buffYw,0)
        self.buffYw = np.append(self.buffYw,0)
        self.buffYw = np.append(self.buffYw,0)
        self.buffYb = np.append(self.buffYb,0)
        self.buffYb = np.append(self.buffYb,0)
        self.buffYb = np.append(self.buffYb,0)
    
        print(self.buffYb)
        print(self.buffYw)
        self.butter_a,self.butter_b = fra.butterworth()


        print("connection established")



    def com_callback(self,ros_img):
        
        k,d =  self._argparse()
        _img = bridge.compressed_imgmsg_to_cv2(ros_img)
        _img = cv2.cvtColor(_img, cv2.COLOR_BGR2RGB)
        _img  = self.pose_estimation(_img, aruco.DICT_5X5_250, k, d)

        cv2.imshow('Image',_img)
        #out.write(_img)
        cv2.waitKey(1)


    def imu_callback(self,data):

        
        self.i_msg_roll = data.orientation.x
        self.i_msg_pitch = data.orientation.y
        self.i_msg_yaw = data.orientation.z 
        #print(data)
        
        
      
    def pose_estimation(self,frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):

        self.start_time = time.perf_counter()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.aruco_dict = aruco.Dictionary_get(aruco_dict_type)
        parameters = cv2.aruco.DetectorParameters_create()
        
        parameters.minCornerDistanceRate = 0.01
        parameters.maxErroneousBitsInBorderRate = 0.8
        
        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(
            gray, cv2.aruco_dict, parameters=parameters)

        #parameters.minDistanceToBorder = 5
        #parameters.maxErroneousBitsInBorderRate = 0.10
        #parameters.cornerRefinementMaxIterations = 300
        #parameters.cornerRefinementMinAccuracy = 0.9
        #parameters.cornerRefinementWinSize = 60
        #parameters.errorCorrectionRate = 0.9
        #parameters.adaptiveThreshWinSizeMax = 50
        #parameters.adaptiveThreshWinSizeMin = 1
        #parameters.adaptiveThreshConstant = 25
        #parameters.cornerRefinementWinSize = 60
        #parameters.polygonalApproxAccuracyRate = 0.01
        #parameters.minOtsuStdDev = 1

        if len(corners) > 0:
            for i in range(0, len(ids)):
                # Estimate pose of each marker and return the values rvec0 and tvec0---(different from those of camera coefficients)
                
                if i == 0 and ids is not None:

                    self.cam_yaw_prsent = True
                            
                    rvec0, tvec0 = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.04, matrix_coefficients,
                                                                                    distortion_coefficients)
                    pos_com = Dronepoints()
                    frm = Frame()

                    scaling_factor = (1/tvec0[0][0][2])

                    pos_com.x = tvec0[0][0][0]/0.767
                    pos_com.z = tvec0[0][0][1]/0.6
                    pos_com.y = tvec0[0][0][2]*-17.2

                    xt = pos_com.x

                    pos_com.roll = rvec0[0][0][1]
                    pos_com.pitch = rvec0[0][0][0]
                    pos_com.yaw = rvec0[0][0][2]
                    #print("yaw before")
                    #print(pos_com.yaw)
                    
                    #pos_com.yaw,pos_com.pitch = (self.filter(pos_com.yaw,pos_com.pitch))
                    #pos_com.yaw = pos_com.yaw
                
                    #pos_com.roll = self.cal_roll((corners[0][0][0])[0],(corners[0][0][0])[1],(corners[0][0][1])[0],(corners[0][0][1])[1])
                    sign = self.cal_yaw_pitch((corners[0][0][0])[0],(corners[0][0][0])[1],(corners[0][0][1])[0],(corners[0][0][1])[1],(corners[0][0][2])[0],(corners[0][0][2])[1],(corners[0][0][3])[0],(corners[0][0][3])[1])
                    speedxy,directxy = self.cal_speed(pos_com.x,pos_com.z)
                    pos_com.yaw,pos_com.pitch = self.filter(pos_com.yaw,pos_com.pitch,sign,speedxy,directxy)
                    pos_com.yaw = self.bufilter(pos_com.yaw)
                    #eyaw,p_eyaw = self.EWMAf(pos_com.yaw)
                    #pos_com.yaw = eyaw
                    '''print("x: "+str(pos_com.x))
                    print("y: "+str(pos_com.y))
                    print("z: "+str(pos_com.z))'''
                    pos_com.x,pos_com.z = frm.xz_estimation(pos_com.x,pos_com.z,pos_com.yaw,pos_com.roll)
                    '''print("roll: "+str(pos_com.roll))
                    print("pitch: "+str(pos_com.pitch))'''
                    print("yaw: "+str(pos_com.yaw))
                    print(f"x: {pos_com.x}")
                    print(f"z: {pos_com.z}")
                    print(f"y: {pos_com.y}")

                    '''print((corners[0][0][0])[0]) #x1
                    print((corners[0][0][0])[1]) #y1 
                    print((corners[0][0][1])[0]) #x2
                    print((corners[0][0][1])[1]) #y2
                    print(pos_com.y)'''#60 - 0.14 55 - 0.155 50 - 0.17 40 - 0.21  30 -0.28  20 - 0.41  

                    with open("data0.txt", "a") as data_file:
                        data_file.write(f"{pos_com.x}\n")
                    with open("data1.txt", "a") as data_file:
                        data_file.write(f"{pos_com.y}\n")
                    with open("data2.txt", "a") as data_file:   
                        data_file.write(f"{pos_com.z}\n")
                    with open("data3.txt", "a") as data_file:  
                        data_file.write(f"{pos_com.roll}\n")
                    with open("data4.txt", "a") as data_file:
                        data_file.write(f"{pos_com.pitch}\n")
                    with open("data5.txt", "a") as data_file:
                        data_file.write(f"{pos_com.yaw}\n")
                    with open("data6.txt", "a") as data_file:
                        data_file.write(f"{xt}\n")
                    
                # Draw a square around the markers
                cv2.aruco.drawDetectedMarkers(frame, corners)
       
                self.pub_pose.publish(pos_com)
                   
        return frame


    def _argparse(self):
        
        ap = argparse.ArgumentParser()
        ap.add_argument("-k", "--K_Matrix", required=True,
                        help="Path to calibration matrix (numpy file)")
        ap.add_argument("-d", "--D_Coeff", required=True,
                        help="Path to distortion coefficients (numpy file)")
        
        args = vars(ap.parse_args())
        
        calibration_matrix_path = args["K_Matrix"]
        distortion_coefficients_path = args["D_Coeff"]
        
        k = np.load(calibration_matrix_path)
        d = np.load(distortion_coefficients_path)

        return k,d

    
    def filter(self,yaw,pitch,sign,speedxy,directxy):
   
        if(pitch) < 0:
            pitch = pitch + 3.14
            pitch = pitch*-1
        else: 
            pitch = pitch - 3.14
            pitch = pitch*-1
        '''if(len(self.cam_yaw)>self.cam_yaw_points-1):            
            self.cam_yaw = np.delete(self.cam_yaw,0)
            self.cam_yaw = np.append(self.cam_yaw,yaw)
        else:
            self.cam_yaw = np.append(self.cam_yaw,yaw)'''     
        yaw = np.abs(yaw)       
        yaw = sign*yaw
        self.yawd = self.yawd + speedxy*directxy
        devn = self.yawd - yaw
   
        return yaw,pitch
    
    
    def cal_roll(self,x1,y1,x2,y2):

        grad = (y1 - y2)/(x2 - x1)        
        ang = np.tan(grad)
        roll = np.arctan(ang)

        return roll


    def cal_yaw_pitch(self,x1,y1,x2,y2,x3,y3,x4,y4):

        hyp1 = np.sqrt(np.square(x1-x2)+np.square(y1-y2))
        hyp2 = np.sqrt(np.square(x2-x3)+np.square(y2-y3))
        hyp3 = np.sqrt(np.square(x3-x4)+np.square(y3-y4))
        hyp4 = np.sqrt(np.square(x4-x1)+np.square(y4-y1))

        '''print("hyp1 "+str(hyp1))
        print("hyp2 "+str(hyp2))
        print("hyp3 "+str(hyp3))
        print("hyp4 "+str(hyp4))'''

        yaw = hyp4 - hyp2
        #pitch = hyp1 - hyp3
        sign = np.sign(yaw)
          
        return sign
    #1.570796326792 radian is equal to 90.000000000057 degrees


    def EWMAf(self,yaw):

        alpha = 0.5
        self.pEWMAy = self.EWMAy
        
        #Apply the EWMA for yaw
        if(len(self.buffY)>self.buf_len):

            self.buffY = np.delete(self.buffY,0)
            self.buffY = np.append(self.buffY,yaw)
            self.EWMAy = 0    

            for i in range (len(self.buffY)-1,-1,-1):
                
                self.EWMAy  = alpha*self.buffY[i] + (1-alpha)*self.EWMAy

        else:

            self.buffY = np.append(self.buffY,yaw)
            self.EWMAy  = alpha*yaw + (1-alpha)*self.EWMAy

        return self.EWMAy,self.pEWMAy



    def cal_speed(self,x,y):

        if(len(self.buffx) > 1):
            self.buffx = np.delete(self.buffx,0)
            self.buffx = np.append(self.buffx,x)
        else:
            self.buffx = np.append(self.buffx,x)

        if(len(self.buffy) > 1):
            self.buffy = np.delete(self.buffy,0)
            self.buffy = np.append(self.buffy,y)
        else:
            self.buffy = np.append(self.buffy,y)

        speedx = self.buffx[0] - self.buffx[1]
        speedy = self.buffy[0] - self.buffy[1]

        mag = np.sqrt(np.square(speedx)+np.square(speedy))
        ang = np.arctan2(speedy,speedx)
        return mag,ang
    


    def bufilter(self,yaw):

        #print(f"butter_a {self.butter_a}")
        #print(f"butter b {self.butter_b}")
        self.buffYw = np.delete(self.buffYw,0)
        self.buffYb = np.delete(self.buffYb,0)
        self.buffYw = np.append(self.buffYw,yaw)
        self.buffYb = np.append(self.buffYb,self.butter_a[1]*self.buffYb[1] + self.butter_a[2]*self.buffYb[0] +self.butter_b[0]*self.buffYw[2]+ self.butter_b[1]*self.buffYw[1] + self.butter_b[2]*self.buffYw[0])

        with open("data7.txt", "a") as data_file:
                        data_file.write(f"{yaw}\n")
        with open("data8.txt", "a") as data_file:
                        data_file.write(f"{self.buffYb[2]}\n")

        return self.buffYb[2]
    

    
    def run(self):

        while not rospy.is_shutdown():

            cv2.destroyAllWindows()
            #print(message)
            rospy.spin()
            #com.ros_rate.sleep()


com = Comm2()

if __name__ == '__main__':

    com._argparse()
    com.run()
    




