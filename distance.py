#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import Int16
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#import matplotlib
#matplotlib.use('Agg')
from matplotlib import pyplot as plt

neutralSteering = 70
bridge = CvBridge()
start_pub = rospy.Publisher("/manual_control/stop_start", Int16)
rospy.sleep(2)
speed_pub = rospy.Publisher("/manual_control/speed", Int16)
rospy.sleep(2)
steering_pub = rospy.Publisher("/manual_control/steering", Int16)
rospy.sleep(2)

def backwards():
 global speed_pub
 speed_pub.publish(900)

def stop():
 global speed_pub
 speed_pub.publish(0)

def forward():
 global speed_pub
 speed_pub.publish(-900)

def obstacleDetection(data, cvImage):
 minDist = 0
 for i in(2*data.height/5, 3*data.height/5):
  for j in(2*data.height/5, 3*data.height/5):
   dist = cvImage[j,i]
   if dist > 0 and (dist < minDist or minDist == 0):
    minDist = dist
 return minDist
 

def callback(data):
 cvImage = bridge.imgmsg_to_cv2(data, "16UC1")
 #passthrough
 dist = obstacleDetection(data, cvImage)
 print(dist)
 if dist > 0 and dist < 500:
  backwards()
 elif dist > 0 and dist < 700:
  stop()
 else:
  forward()

 


rospy.init_node("distance_node")
depth_sub = rospy.Subscriber("/app/camera/depth/image_raw", Image, callback)
start_pub.publish(0)
rospy.sleep(5)
start_pub.publish(0)
steering_pub.publish(neutralSteering)
rospy.spin()