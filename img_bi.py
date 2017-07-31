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

class image_converter:

  def __init__(self):
    self.rgb_pub = rospy.Publisher("/image_processing/bin_rgb",Image)
    self.hsv_pub = rospy.Publisher("/image_processing/bin_hsv",Image)
    self.yuv_pub = rospy.Publisher("/image_processing/bin_yuv",Image)
    self.gray_pub = rospy.Publisher("/image_processing/bin_gray",Image, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/app/camera/rgb/image_raw",Image,self.callback, queue_size=1)


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)


    #make it gray
    gray=cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    #bi_gray
    bi_gray_max = 255
    bi_gray_min = 245
    ret,thresh1=cv2.threshold(gray, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY);

    try:
      self.gray_pub.publish(self.bridge.cv2_to_imgmsg(thresh1, "mono8"))
    except CvBridgeError as e:
      print(e)


    #bi_rgb
    r_max = 244;
    r_min = 0;
    g_max = 255;
    g_min = 0;
    b_max = 255;
    b_min = 0;
    b,g,r = cv2.split(cv_image)
    for j in range(cv_image.shape[0]):
      for i in range(cv_image.shape[1]):
        if (r[j,i] >= r_min and r[j,i] <= r_max):
          if (g[j,i] >= g_min and g[j,i] <= g_max):
            if (b[j,i] >= b_min and b[j,i] <= b_max):
              r[j,i]=0
              g[j,i]=0
              b[j,i]=0
            else:
              r[j,i]=255
              g[j,i]=255
              b[j,i]=255
    bi_rgb = (r+b+g)
    self.rgb_pub.publish(self.bridge.cv2_to_imgmsg(bi_rgb, "mono8"))

    #bi_hsv
    h_max = 255;
    h_min = 0;
    s_max = 255;
    s_min= 0;
    v_max = 252;
    v_min = 0;
    hsv=cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV);
    h,s,v = cv2.split(hsv)

    for j in xrange(hsv.shape[0]):
      for i in xrange(hsv.shape[1]):
        if  (v[j,i]>= v_min and v[j,i]<= v_max and s[j,i]>= s_min and s[j,i]<= s_max and h[j,i]>= h_min and h[j,i]<= h_max):
          h[j,i]=0
          s[j,i]=0
          v[j,i]=0
        else:
          h[j,i]=255
          s[j,i]=255
          v[j,i]=255

    bi_hsv = v
    self.hsv_pub.publish(self.bridge.cv2_to_imgmsg(bi_hsv, "mono8"))

    #bi_yuv
    yuv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2YUV)
    y,u,v = cv2.split(yuv)

    y_min = 0
    y_max = 200


    for j in range(cv_image.shape[0]):
      for i in range(cv_image.shape[1]):
        if (y[j,i] >= y_min and y[j,i] <= y_max):
          y[j,i]=0
        else:
          y[j,i]=255


    bi_yuv = y
    self.yuv_pub.publish(self.bridge.cv2_to_imgmsg(bi_yuv, "mono8"))


def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
  main(sys.argv)
