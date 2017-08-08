#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
import math
import matplotlib.pyplot as plt
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sklearn import linear_model
####################################
#Entwicklungsstatus: Fertig, funktioniert.
#Die RGB-Maske sollte je nach Lichtverhaeltnissen leicht angepasst werden.
#Leonardo Balestrieri
#Mika Delor
#Tom Guelenman
####################################
#die "realen" Werte der Ballons
blue,purple,red,green = (3.57,1.15),(2.33,3.0),(3.57,3.0),(2.33,1.15)

def callback(data):
  global blue, purple, red, green
  
  bridge = CvBridge()
  cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
  
  #gruen suchen
  r_min_gruen,g_min_gruen,b_min_gruen = 0, 86, 24 
  r_max_gruen,g_max_gruen,b_max_gruen = 41,127,70 
  #blau suchen
  r_min_blau,g_min_blau,b_min_blau = 0,31,130
  r_max_blau,g_max_blau,b_max_blau = 74,116,251

  #lila suchen
  r_min_lila,g_min_lila,b_min_lila = 151,46,254
  r_max_lila,g_max_lila,b_max_lila = 213,108,255 

  #rot suchen
  r_min_rot,g_min_rot,b_min_rot = 73,0,0 
  r_max_rot,g_max_rot,b_max_rot = 121,0,0 
  
  #arrays initialisieren
  cluster_gr_x = []
  cluster_gr_y = []
  cluster_b_x = []
  cluster_b_y = []
  cluster_l_x = []
  cluster_l_y = []
  cluster_r_x = []
  cluster_r_y = []

  cluster_gr =[]
  cluster_b =[]
  cluster_l =[]
  cluster_r =[]


  #matrix spliten
  b,g,r = cv2.split(np.array(cv_image))

  #schleife um farben zu finden
  for i in range(0, len(r)):
    #len(r[0]) gibt die anzahl der Elemente in der Matrix zurueck, die in der x Koordinate liegen
    for j in range(0, len(r[0])):
      #gruen suchen
      if (b[i][j] >= b_min_gruen and b[i][j] <= b_max_gruen) and (r[i][j] >= r_min_gruen and r[i][j] <= r_max_gruen) and (g[i][j] >= g_min_gruen and g[i][j] <= g_max_gruen):
        #gruenen pixel zum cluster hinzufuegen
        cluster_gr_x.append(i)
        cluster_gr_y.append(j)
        cluster_gr.append((i,j)) 

      #blau suchen
      if (b[i][j] >= b_min_blau and b[i][j] <= b_max_blau) and (r[i][j] >= r_min_blau and r[i][j] <= r_max_blau) and (g[i][j] >= g_min_blau and g[i][j] <= g_max_blau):
        #blauen pixel zum cluster hinzufuegen
        cluster_b_x.append(i)
        cluster_b_y.append(j)
        cluster_b.append((i,j))

      #lila suchen
      if (b[i][j] >= b_min_lila and b[i][j] <= b_max_lila) and (r[i][j] >= r_min_lila and r[i][j] <= r_max_lila) and (g[i][j] >= g_min_lila and g[i][j] <= g_max_lila):
        #lila pixel zum cluster hinzufuegen
        cluster_l_x.append(i)
        cluster_l_y.append(j)
        cluster_l.append((i,j))

      #rot suchen
      if (b[i][j] >= b_min_rot and b[i][j] <= b_max_rot) and (r[i][j] >= r_min_rot and r[i][j] <= r_max_rot) and (g[i][j] >= g_min_rot and g[i][j] <= g_max_rot):
        #roten pixel zum cluster hinzufuegen
        cluster_r_x.append(i)
        cluster_r_y.append(j)
        cluster_r.append((i,j))


  #mittelpunkt der x und y koordinaten berechnen
  gr_x_center = sum(cluster_gr_x)/len(cluster_gr_x)
  gr_y_center = sum(cluster_gr_y)/len(cluster_gr_y)
  b_x_center = sum(cluster_b_x)/len(cluster_b_x)
  b_y_center = sum(cluster_b_y)/len(cluster_b_y)
  l_x_center = sum(cluster_l_x)/len(cluster_l_x)
  l_y_center = sum(cluster_l_y)/len(cluster_l_y)
  r_x_center = sum(cluster_r_x)/len(cluster_r_x)
  r_y_center = sum(cluster_r_y)/len(cluster_r_y)

  #...und endgueltige Position der Ballons festlegen
  ballon_gruen = (gr_x_center,gr_y_center)
  ballon_blau = (b_x_center,b_y_center)
  ballon_lila = (l_x_center,l_y_center)
  ballon_rot = (r_x_center,r_y_center)

  #unser auto ist in der mitte der kamera
  car_position = (240,320)

  #winkel zwischen gelb und gruen:
  w_l_gr = winkel(ballon_lila,ballon_gruen,car_position)
  w_r_b = winkel(ballon_rot,ballon_blau,car_position)
  w_b_gr = winkel(ballon_blau,ballon_gruen,car_position)
  w_r_l = winkel(ballon_rot,ballon_lila,car_position)
  w_r_gr = winkel(ballon_rot,ballon_gruen,car_position)
  w_l_b = winkel(ballon_lila,ballon_blau,car_position)

  #kreise berechnen
  liste_aller_kreise = []  

  if not(kreis_berechnen(purple, green, kreiswinkel_berechnen(w_l_gr)) == None):
    kreis_l_gr_1, kreis_l_gr_2  = kreis_berechnen(purple, green, kreiswinkel_berechnen(w_l_gr))
    liste_aller_kreise.append(kreis_l_gr_1)
    liste_aller_kreise.append(kreis_l_gr_2)

  if not(kreis_berechnen(purple, green, kreiswinkel_berechnen(w_r_b)) == None):
    kreis_r_b_1, kreis_r_b_2 = kreis_berechnen(red, blue, kreiswinkel_berechnen(w_r_b))

    liste_aller_kreise.append(kreis_r_b_1)
    liste_aller_kreise.append(kreis_r_b_2)

  liste_aller_schnittpunkte = []
  for i in range(len(liste_aller_kreise)-2):
  
    if i%2==0:
      k = i+2
    else:
      k = i+1
    for j in range(k,len(liste_aller_kreise)):
      if not (schnittpunkte_berechnen(liste_aller_kreise[i], liste_aller_kreise[j]) == None):
        s1, s2 = schnittpunkte_berechnen(liste_aller_kreise[i], liste_aller_kreise[j])
      
        liste_aller_schnittpunkte.append(s1)
        liste_aller_schnittpunkte.append(s2)

  toleranz_grenze = 0.05 #in Meter
  for i in range(len(liste_aller_schnittpunkte)):
    for j in range(i+1,len(liste_aller_schnittpunkte)):
      abstand_i_j = abstand_berechnen(liste_aller_schnittpunkte[i],liste_aller_schnittpunkte[j])
      if abstand_i_j <= toleranz_grenze:
        liste_aller_schnittpunkte[j] = liste_aller_schnittpunkte[i]
        

  haeufigkeiten = []
  for i in range(len(liste_aller_schnittpunkte)):
    haeufigkeiten.append(liste_aller_schnittpunkte.count(liste_aller_schnittpunkte[i])) 
  max_index = haeufigkeiten.index(max(haeufigkeiten))
  maximale_vorkommnisse = []
  for i in range(len(haeufigkeiten)):
    if haeufigkeiten[i] == haeufigkeiten[max_index] and liste_aller_schnittpunkte[i] not in maximale_vorkommnisse:
      maximale_vorkommnisse.append(liste_aller_schnittpunkte[i])
  
  bild_zentrum = car_position
  koordinaten_ursprung = (r_x_center,r_y_center)
  bild_zentrum_t = (bild_zentrum[0] - koordinaten_ursprung[0],bild_zentrum[1] - koordinaten_ursprung[1])
  b_x_center_t = b_x_center - koordinaten_ursprung[0]
  b_y_center_t = b_y_center - koordinaten_ursprung[1]
  l_x_center_t = l_x_center - koordinaten_ursprung[0]
  l_y_center_t = l_y_center - koordinaten_ursprung[1]
  koordinate_x_t = (skalarprodukt(bild_zentrum_t,(l_x_center_t,l_y_center_t)))/(norm((l_x_center_t,l_y_center_t))**2)
  koordinate_y_t = (skalarprodukt(bild_zentrum_t,(b_x_center_t,b_y_center_t)))/(norm((b_x_center_t,b_y_center_t))**2)
  k_position_vermutung_x = 3.57 - koordinate_x_t * 1.24
  k_position_vermutung_y = 3.0 - koordinate_y_t * 1.85
  vermutung = (k_position_vermutung_x,k_position_vermutung_y)
  
  interessanter_index = 0
  gesuchtes_min = abstand_berechnen(maximale_vorkommnisse[0],vermutung)
  for i in range (len(maximale_vorkommnisse)):
    if abstand_berechnen(maximale_vorkommnisse[i],vermutung) < gesuchtes_min:
      interessanter_index = i
      gesuchtes_min = abstand_berechnen(maximale_vorkommnisse[i],vermutung)
  print("ENDERGEBNIS: ", maximale_vorkommnisse[interessanter_index])
  return(maximale_vorkommnisse[interessanter_index])

  plt.plot([gr_x_center,b_x_center,l_x_center,r_x_center,240],[gr_y_center,b_y_center,l_y_center,r_y_center, 320], 'ro')
  plt.axis([0,480,0,640])
  plt.show()

def abstand_berechnen(punkt1,punkt2):
  diff_x = punkt1[0] - punkt2[0]
  diff_y = punkt1[1] - punkt2[1]
  return norm((diff_x,diff_y)) 
  
def schnittpunkte_berechnen(kreis1,kreis2):
  erste_punkte = []
  zweite_punkte = []
  for q in range(10):
    #die schnittpunkte funktion gibt zwei schnittpunkte zurueck
    if not(schnittpunkte_der_kreise(kreis1, kreis2) == None):
      punkt1, punkt2 = schnittpunkte_der_kreise(kreis1, kreis2)
      #jeweils die ersten und die zweiten speichern wir in verschiedenen listen ab
      erste_punkte.append(punkt1)
      zweite_punkte.append(punkt2)
      #und spliten diese jetzt noch in x - und y - koordinaten - listen
      erste_punkte_x_liste, erste_punkte_y_liste = zip(*erste_punkte)
      zweite_punkte_x_liste, zweite_punkte_y_liste = zip(*zweite_punkte)
      #und berechnen dann davon den durchschnitt
      finaler_punkt_1_x = (sum(erste_punkte_x_liste)/len(erste_punkte_x_liste))
      finaler_punkt_1_y = (sum(erste_punkte_y_liste)/len(erste_punkte_y_liste))
      finaler_punkt_2_x = (sum(zweite_punkte_x_liste)/len(zweite_punkte_x_liste))
      finaler_punkt_2_y = (sum(zweite_punkte_y_liste)/len(zweite_punkte_y_liste))
      #fuegen diese durchschnitte wieder zu richtigen punkten zusammen
      finaler_punkt_1 = (finaler_punkt_1_x,finaler_punkt_1_y)
      finaler_punkt_2 = (finaler_punkt_2_x,finaler_punkt_2_y)
      #und geben diese aus / zurueck
      return finaler_punkt_1, finaler_punkt_2
  return None



def schnittpunkte_der_kreise(kreis1, kreis2):
  
  x1,y1 = kreis1[0]
  r1 = kreis1[1]
  x2,y2 = kreis2[0]
  r2 = kreis2[1]

  dx,dy = x2-x1,y2-y1
  d = math.sqrt(dx*dx+dy*dy)
  if d > r1+r2:
    return None # no solutions, the circles are separate
  if d < abs(r1-r2):
    return None # no solutions because one circle is contained within the other
  if d == 0 and r1 == r2:
    return None # circles are coincident and there are an infinite number of solutions

  a = (r1*r1-r2*r2+d*d)/(2*d)
  h = math.sqrt(r1*r1-a*a)
  xm = x1 + a*dx/d
  ym = y1 + a*dy/d
  xs1 = xm + h*dy/d
  xs2 = xm - h*dy/d
  ys1 = ym - h*dx/d
  ys2 = ym + h*dx/d
  return (xs1,ys1),(xs2,ys2)

#Winkelformel wurde angepasst-ist sie wirklich richtig? Ja.
def kreiswinkel_berechnen(winkel):
  if winkel < 90:
    return (90 - winkel)
  else:
    #wurde nachgerechnet, stimmt. 07.August '17
    return (winkel - 90)
#fuer zwei Ballons gibt es leider zwei potentielle Kreise;
#die Methode kreis_berechnen berechnet beide potentielle Kreise

def kreis_berechnen(punkt1, punkt2, winkel):
  #Funktion geaendert 07.August'17
  ankathete = (abstand_berechnen(punkt1,punkt2))/2
  radius = ankathete/(math.cos(math.radians(winkel)))
  if not(schnittpunkte_berechnen((punkt1,radius),(punkt2,radius)) == None):
    mittelpunkt_1, mittelpunkt_2 = schnittpunkte_berechnen((punkt1,radius),(punkt2,radius))
    return (mittelpunkt_1,radius), (mittelpunkt_2,radius)
  return None  

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
  return math.degrees(math.acos(skalarprodukt(vektor1,vektor2) / (norm(vektor1) * norm(vektor2))))

#das skalarprodukt ausrechnen
def skalarprodukt(vektor1, vektor2):
  return (vektor1[0] * vektor2[0] + vektor1[1] * vektor2[1])

#die norm ausrechnen
def norm(vektor):
  return math.sqrt(vektor[0]**2 + vektor[1]**2)

rospy.init_node('gps_node', anonymous=True)
ransac_sub = rospy.Subscriber("/usb_cam/image_raw",Image,callback, queue_size=1)
rospy.spin()

