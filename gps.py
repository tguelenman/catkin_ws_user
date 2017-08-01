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
####################################
#unsere Loesung laesst sich nicht richtig ausfuehren, aber wir hoffen, dass Loesungsansatze mit
#den vorhandenen Kommentaren ausreichen, um einige Punkte zu holen.
#wir haben die Winkel zwischen den Ballons und der aktuellen Position ausgerechnet.
#Was uns noch fehlt: -mithilfe des Winkels und 2 Ballons finden wir heraus, welchen Kreis diese Werte ergeben
#drei sich daraus ergebende Kreise bilden unseren eindeutigen Schnittpunkt
####################################
#die "realen" Werte der Ballons
global blue,yellow,black,red,green = (0,0),(2.35,0),(0.48,1.0),(2.35,1.9),(0,1.9)

rospy.init_node('gps_node', anonymous=True)
ransac_sub = rospy.Subscriber("/usb_cam/image_raw",Image,callback, queue_size=1)
rospy.spin()

def callback(data):

  img = data.data
  
  #gruen suchen
  r_min_gruen,g_min_gruen,b_min_gruen = 0,90,0
  r_max_gruen,g_max_gruen,b_max_gruen = 30,255,34

  #blau suchen
  r_min_blau,g_min_blau,b_min_blau = 0,0,21
  r_max_blau,g_max_blau,b_max_blau = 30,186,255

  #gelb suchen
  r_min_gelb,g_min_gelb,b_min_gelb = 164,166,68
  r_max_gelb,g_max_gelb,b_max_gelb = 223,230,145

  #rot suchen
  r_min_rot,g_min_rot,b_min_rot = 120,0,0
  r_max_rot,g_max_rot,b_max_rot = 255,65,90

  #arrays initialisieren
  cluster_gr_x = []
  cluster_gr_y = []
  cluster_b_x = []
  cluster_b_y = []
  cluster_ge_x = []
  cluster_ge_y = []
  cluster_r_x = []
  cluster_r_y = []

  #matrix spliten
  b,g,r = cv2.split(cv_image)

  #schleife um farben zu finden
  for i in range(0,640):
    for j in range(0,480):
      #gruen suchen
      if (b[i][j] >= b_min_gruen and b[i][j] <= b_max_gruen) and (r[i][j] >= r_min_gruen and r[i][j] <= r_max_gruen) and (g[i][j] >= g_min_gruen and g[i][j] <= g_max_gruen):
        #gruenen pixel zum cluster hinzufuegen
        cluster_gr_x.append(i)
        cluster_gr_y.append(j)
      #blau suchen
      if (b[i][j] >= b_min_blau and b[i][j] <= b_max_blau) and (r[i][j] >= r_min_blau and r[i][j] <= r_max_blau) and (g[i][j] >= g_min_blau and g[i][j] <= g_max_blau):
        #blauen pixel zum cluster hinzufuegen
        cluster_b_x.append(i)
        cluster_b_y.append(j)
      #gelb suchen
      if (b[i][j] >= b_min_gelb and b[i][j] <= b_max_gelb) and (r[i][j] >= r_min_gelb and r[i][j] <= r_max_gelb) and (g[i][j] >= g_min_gelb and g[i][j] <= g_max_gelb):
        #gelben pixel zum cluster hinzufuegen
        cluster_ge_x.append(i)
        cluster_ge_y.append(j)
      #rot suchen
      if (b[i][j] >= b_min_rot and b[i][j] <= b_max_rot) and (r[i][j] >= r_min_rot and r[i][j] <= r_max_rot) and (g[i][j] >= g_min_rot and g[i][j] <= g_max_rot):
        #roten pixel zum cluster hinzufuegen
        cluster_r_x.append(i)
        cluster_r_y.append(j)

  #mittelpunkt der x und y koordinaten berechnen
  gr_x_center = sum(cluster_gr_x)/len(cluster_gr_x)
  gr_y_center = sum(cluster_gr_y)/len(cluster_gr_y)
  b_x_center = sum(cluster_b_x)/len(cluster_b_x)
  b_y_center = sum(cluster_b_y)/len(cluster_b_y)
  ge_x_center = sum(cluster_ge_x)/len(cluster_ge_x)
  ge_y_center = sum(cluster_ge_y)/len(cluster_ge_y)
  r_x_center = sum(cluster_r_x)/len(cluster_r_x)
  r_y_center = sum(cluster_r_y)/len(cluster_r_y)

  #...und endgueltige Position der Ballons festlegen
  ballon_gruen = (gr_x_center,gr_y_center)
  ballon_blau = (b_x_center,b_y_center)
  ballon_gelb = (ge_x_center,ge_y_center)
  ballon_rot = (r_x_center,r_y_center)

  #unser auto ist in der mitte der kamera
  car_position = (320,240)

  #winkel zwischen gelb und gruen:
  w_ge_gr = winkel(ballon_gelb,ballon_gruen,car_position)
  w_r_b = winkel(ballon_rot,ballon_blau,car_position)
  w_r_ge = winkel(ballon_rot,ballon_gelb,car_position)


  


#den winkel von unserer position aus zu 2 ballons ausrechnen
def winkel(ballon1, ballon2, position):
  ballon1_x = ballon1[0]
  ballon1_y = ballon1[1]
  ballon2_x = ballon2[0]
  ballon2_y = ballon2[1]
  position_x = position[0]
  position_y = position[1]

  #vektor 1 ausrechnen
  vektor1 = (ballon1_x - position_x, ballon1_y - position_y)
  #vektor 2 ausrechnen
  vektor2 = (ballon2_x - position_x, ballon2_y - position_y)

  #verbindungsvektor berechnen
  return math.acos(skalarprodukt(vektor1,vektor2) / (norm(vektor1) * norm(vektor2))

#das skalarprodukt ausrechnen
def skalarprodukt(vektor1,vektor2):
  return (vektor1[0] * vektor2[0] + vektor1[1] * vektor2[1])

#die norm ausrechnen
def norm(vektor):
  return math.sqrt(vektor[0]**2 + vektor[1]**2)

