#!/usr/bin/env python3

import numpy as np
from scipy import signal
import matplotlib.pyplot as plt
import math

class Frame:

    def __init__(self):

        self.x_axis = 0
        self.y_axis = 0
        self.z_axis = 0
        self.yaw = 0

        self.x_vel = 0
        self.y_vel = 0
        self.z_vel = 0


    def velocity(self,xcr,ycr,zcr,ywcr,rlcr,xst,yst,zst,ywst):

        #print(f"xcr: {xcr}, ycr: {ycr}, zcr: {zcr}, ywcr: {ywcr}, rlcr: {rlcr}")
        rxcr,rzcr = self.xz_estimation(xcr,zcr,ywcr,rlcr)

        x_vel_comp = xst - rxcr
        z_vel_comp = zst - rzcr
        y_vel_comp = yst - ycr

        comb_vel_comp = np.sqrt(np.square(x_vel_comp)+np.square(y_vel_comp))
        comb_ang_comp = np.arctan2(y_vel_comp,x_vel_comp)
        yaw_rate = ywst - ywcr
    
        #x_vel = comb_vel_comp*np.cos(comb_ang_comp-ywcr) 
        #y_vel = comb_vel_comp*np.sin(comb_ang_comp-ywcr)
        #--sim
        x_vel = comb_vel_comp*np.cos(comb_ang_comp)
        y_vel = comb_vel_comp*np.sin(comb_ang_comp)
        #--sim
        z_vel = z_vel_comp

        return x_vel, y_vel, z_vel, yaw_rate
     
    
    def xz_estimation(self,xcr,zcr,ywcr,rlcr):

        r = -1*ywcr 
        
        #rxcr = xcr - r*np.cos(rlcr)
        #rzcr = zcr + r*np.sin(rlcr)

        #rxcr = xcr - r*0.2
        #--sim
        rxcr = xcr
        #--sim
        rzcr = zcr 

        return rxcr, rzcr


    def butterworth(self):

        wc = 2*np.pi*0.5 # cutoff frequency (rad/s)
        n = 2 # Filter order

        # Compute the Butterworth filter coefficents
        a = np.zeros(n+1)
        gamma = np.pi/(2.0*n)
        a[0] = 1 # first coef is always 1
        for k in range(0,n):
            rfac = np.cos(k*gamma)/np.sin((k+1)*gamma)
            a[k+1] = rfac*a[k] # Other coefficients by recursion

        print("Butterworth polynomial coefficients a_i:                " + str(a))

        # Adjust the cutoff frequency
        c = np.zeros(n+1)
        for k in range(0,n+1):
            c[n-k] = a[k]/pow(wc,k)

        print("Butterworth coefficients with frequency adjustment c_i: " + str(c))

        num = [1]      # transfer function numerator coefficients
        den = c        # transfer function denominator coefficients
        lowPass = signal.TransferFunction(num,den) # Transfer function

        dt = 1.0/20
        discreteLowPass = lowPass.to_discrete(dt,method='gbt',alpha=0.5)
        print(discreteLowPass)

        # The coefficients from the discrete form of the filter transfer function (but with a negative sign)
        b = discreteLowPass.num
        a = -discreteLowPass.den
        print("Filter coefficients b_i: " + str(b))
        print("Filter coefficients a_i: " + str(a[1:]))

        return a,b








        



        

        



        





        
         
    

        
        







