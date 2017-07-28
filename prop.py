#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String, Float32, Int16
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import matplotlib
#matplotlib.use('Agg')
from matplotlib import pyplot as plt

desiredHeading = None
more_than_once = False
calibratedZeroSteeringAngle = 70
Kp = 2

def callback(degree_msg):

  global more_than_once, calibratedZeroSteeringAngle, Kp, desiredHeading

  degree = degree_msg.data
  #neue werte sammeln
  currentHeading = degree
  if not(more_than_once):
    #hier steht immer unser richtwert
    desiredHeading = degree
    more_than_once = True

  diff = desiredHeading - currentHeading
  control_pub = rospy.Publisher("/manual_control/steering",Int16)
  u = Kp * diff + calibratedZeroSteeringAngle
  control_pub.publish(u)


print("hi")
rospy.init_node('prop_c', anonymous=True)
start_pub = rospy.Publisher("/manual_control/stop_start",Int16)
start_pub.publish(0)
print("hi2")
rospy.sleep(1)
speed_pub = rospy.Publisher("/manual_control/speed",Int16)
speed_pub.publish(300)
rospy.sleep(1)

degree_sub = rospy.Subscriber("/model_car/yaw",Float32,callback)

rospy.spin()
