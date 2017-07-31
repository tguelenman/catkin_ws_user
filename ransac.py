#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import ransac
from sklearn import linear_model


rospy.init_node('ransac_node', anonymous=True)
ransac_sub = rospy.Subscriber("/image_processing/bin_yuv",Image,callback, queue_size=1)
rospy.spin()

def callback(data):
  #matrix
  img = data.data
  
  #bild halbieren
  img_first_half = [][]
  img_second_half = [][]
  array_x = []
  array_y = []
  array_x2 = []
  array_y2 = []

  #erste haelfte
  for x in range (0,640):
    for y in range(0, 240):
      img_first_half[i][j] = img[i][j]
      if img[i][j] > 0:
        array_x.append(x)
        array_y.append(y)

  #zweite haelfte
  for x in range (0,640):
    for y in range(241, 480):
      img_second_half[i][j] = img[i][j]
      if img[i][j] > 0:
        array_x2.append(x)
        array_y2.append(y)
  

  print(RANSACRegressor.fit(array_x, array_y))
  print(RANSACRegressor.fit(array_x2, array_y2))
