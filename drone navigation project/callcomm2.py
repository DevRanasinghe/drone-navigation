#!/usr/bin/env python3


import numpy as np
import cv2
import sys
from cv2 import aruco
import argparse
import time
import rospy
import math
from dynamic_reconfigure.client import Client


import comm2

ar = comm2.Comm2

k,d = ar._argparse()

ar.run()

#ar.update_parameters()