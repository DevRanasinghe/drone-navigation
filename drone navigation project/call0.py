import numpy as np
import cv2
import sys
from cv2 import aruco
import argparse
import time
import rospy
import math


import call

ar = call.Callback()

k,d,IP = ar._argparse(aruco.DICT_6X6_250)

ar.show_video('https://'+IP+':8080/video',aruco.DICT_6X6_250,k,d)








