import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
import math
import matplotlib.pyplot as plt
from std_msgs.msg import String, Int16, Float32
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
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
punkte_array = []
punkte_array_index = 0
aktueller_yaw = 0
start_yaw = 0
aktuelles_steering = 85
einmal_ausgefuehrt = False
gps_printen = 0

def callback(data):
  
  #######################GPS#######################
  global blue, purple, red, green, punkte_array, punkte_array_index, aktueller_yaw, start_yaw, gps_printen, aktuelles_steering 
  
  bridge = CvBridge()
  cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
  
  boundaries = [
    #red
    ([0,0,108],[69,66,219]),
    #gruen
    ([13,137,37],[90,192,92]),
    #blau
    ([101,19,0],[183,92,43]),
    #lila
    ([188,57,134],[255,102,206])
  ]
  

  indices_x = [0]*4
  indices_y = [0]*4

  
  #r,g,b,l
  fund_liste = [True]*4
  anzahl_gefunden = 4
  
  rot_gefunden = True
  gruen_gefunden = True
  blau_gefunden = True
  lila_gefunden = True

  for k, (lower, upper) in enumerate(boundaries):
	
    lower = np.array(lower, dtype = "uint8")
    upper = np.array(upper, dtype = "uint8")
    mask = cv2.inRange(cv_image, lower, upper)
    (indices_x[k], indices_y[k]) = np.nonzero(mask)
    if len(indices_x[k]) == 0:
      fund_liste[k] = False
      anzahl_gefunden -=1
  
  if not fund_liste[0]:
    rot_gefunden = False

  if not fund_liste[1]:
    gruen_gefunden = False
 
  if not fund_liste[2]:
    blau_gefunden = False

  if not fund_liste[3]:
    lila_gefunden = False

  r_x_center = np.mean(indices_x[0])
  r_y_center = np.mean(indices_y[0])

  gr_x_center = np.mean(indices_x[1])
  gr_y_center = np.mean(indices_y[1])

  b_x_center = np.mean(indices_x[2])
  b_y_center = np.mean(indices_y[2])

  l_x_center = np.mean(indices_x[3])
  l_y_center = np.mean(indices_y[3])

  if gruen_gefunden:
    ballon_gruen = (gr_x_center,gr_y_center)
  if blau_gefunden:
    ballon_blau = (b_x_center,b_y_center)
  if lila_gefunden:
    ballon_lila = (l_x_center,l_y_center)
  if rot_gefunden:
    ballon_rot = (r_x_center,r_y_center)

  #print( "G: ",ballon_gruen,"B: ",ballon_blau,"L: ",ballon_lila,"R: ",ballon_rot)
  #unser auto ist in der mitte der kamera
  car_position = (240,320)
  
  #kreise berechnen#####################################################
  liste_aller_kreise = []  

  #winkel zw lila und gruen
  if blau_gefunden == False:
   
    w_l_gr = winkel(ballon_lila,ballon_gruen,car_position)
    #if not(kreis_berechnen(purple, green, kreiswinkel_berechnen(w_l_gr)) == None):
    kreis_l_gr_1, kreis_l_gr_2  = kreis_berechnen(purple, green, kreiswinkel_berechnen(w_l_gr))
    liste_aller_kreise.append(kreis_l_gr_1)
    liste_aller_kreise.append(kreis_l_gr_2)
   
    w_r_l = winkel(ballon_rot,ballon_lila,car_position)
    #if not(kreis_berechnen(red, purple, kreiswinkel_berechnen(w_r_l)) == None):
    kreis_r_l_1, kreis_r_l_2 = kreis_berechnen(red, purple, kreiswinkel_berechnen(w_r_l))
    liste_aller_kreise.append(kreis_r_l_1)
    liste_aller_kreise.append(kreis_r_l_2)
  
  elif gruen_gefunden == False:
    w_r_l = winkel(ballon_rot,ballon_lila,car_position)
    #if not(kreis_berechnen(red, purple, kreiswinkel_berechnen(w_r_l)) == None):
    kreis_r_l_1, kreis_r_l_2 = kreis_berechnen(red, purple, kreiswinkel_berechnen(w_r_l))
    liste_aller_kreise.append(kreis_r_l_1)
    liste_aller_kreise.append(kreis_r_l_2)

    w_r_b = winkel(ballon_rot,ballon_blau,car_position)
    #if not(kreis_berechnen(red, blue, kreiswinkel_berechnen(w_r_b)) == None):
    kreis_r_b_1, kreis_r_b_2 = kreis_berechnen(red, blue, kreiswinkel_berechnen(w_r_b))
    liste_aller_kreise.append(kreis_r_b_1)
    liste_aller_kreise.append(kreis_r_b_2)

  elif rot_gefunden == False:
    
    w_l_gr = winkel(ballon_lila,ballon_gruen,car_position)
    #if not(kreis_berechnen(purple, green, kreiswinkel_berechnen(w_l_gr)) == None):
    kreis_l_gr_1, kreis_l_gr_2  = kreis_berechnen(purple, green, kreiswinkel_berechnen(w_l_gr))
    liste_aller_kreise.append(kreis_l_gr_1)
    liste_aller_kreise.append(kreis_l_gr_2)

    w_b_gr = winkel(ballon_blau,ballon_gruen,car_position)
    #if not(kreis_berechnen(green, blue, kreiswinkel_berechnen(w_b_gr)) == None):
    kreis_b_gr_1, kreis_b_gr_2 = kreis_berechnen(green, blue, kreiswinkel_berechnen(w_b_gr))
    liste_aller_kreise.append(kreis_b_gr_1)
    liste_aller_kreise.append(kreis_b_gr_2)

  else:
  
    w_r_b = winkel(ballon_rot,ballon_blau,car_position)
    #if not(kreis_berechnen(red, blue, kreiswinkel_berechnen(w_r_b)) == None):
    kreis_r_b_1, kreis_r_b_2 = kreis_berechnen(red, blue, kreiswinkel_berechnen(w_r_b))
    liste_aller_kreise.append(kreis_r_b_1)
    liste_aller_kreise.append(kreis_r_b_2)

   
    w_b_gr = winkel(ballon_blau,ballon_gruen,car_position)
    #if not(kreis_berechnen(green, blue, kreiswinkel_berechnen(w_b_gr)) == None):
    kreis_b_gr_1, kreis_b_gr_2 = kreis_berechnen(green, blue, kreiswinkel_berechnen(w_b_gr))
    liste_aller_kreise.append(kreis_b_gr_1)
    liste_aller_kreise.append(kreis_b_gr_2)
  

  liste_aller_schnittpunkte = []
  for i in range(len(liste_aller_kreise)-2):
  
    if i%2==0:
      k = i+2
    else:
      k = i+1
    for j in range(k,len(liste_aller_kreise)):
      if not (schnittpunkte_der_kreise(liste_aller_kreise[i], liste_aller_kreise[j]) == None):
        s1, s2 = schnittpunkte_der_kreise(liste_aller_kreise[i], liste_aller_kreise[j])
      
        liste_aller_schnittpunkte.append(s1)
        liste_aller_schnittpunkte.append(s2)
  bild_zentrum = car_position

  if gruen_gefunden == False:
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
  
  elif blau_gefunden == False:
    koordinaten_ursprung = (l_x_center,l_y_center)
    bild_zentrum_t = (bild_zentrum[0] - koordinaten_ursprung[0],bild_zentrum[1] - koordinaten_ursprung[1])
    gr_x_center_t = gr_x_center - koordinaten_ursprung[0]
    gr_y_center_t = gr_y_center - koordinaten_ursprung[1]
    r_x_center_t = r_x_center - koordinaten_ursprung[0]
    r_y_center_t = r_y_center - koordinaten_ursprung[1]
    koordinate_x_t = (skalarprodukt(bild_zentrum_t,(gr_x_center_t,gr_y_center_t)))/(norm((gr_x_center_t,gr_y_center_t))**2)
    koordinate_y_t = (skalarprodukt(bild_zentrum_t,(r_x_center_t,r_y_center_t)))/(norm((r_x_center_t,r_y_center_t))**2)
    k_position_vermutung_x = 2.33 + koordinate_x_t * 1.24
    k_position_vermutung_y = 3.0 - koordinate_y_t * 1.85
    vermutung = (k_position_vermutung_x,k_position_vermutung_y)

  elif rot_gefunden == False:
    koordinaten_ursprung = (gr_x_center,gr_y_center)
    bild_zentrum_t = (bild_zentrum[0] - koordinaten_ursprung[0],bild_zentrum[1] - koordinaten_ursprung[1])
    b_x_center_t = b_x_center - koordinaten_ursprung[0]
    b_y_center_t = b_y_center - koordinaten_ursprung[1]
    l_x_center_t = l_x_center - koordinaten_ursprung[0]
    l_y_center_t = l_y_center - koordinaten_ursprung[1]
    koordinate_x_t = (skalarprodukt(bild_zentrum_t,(l_x_center_t,l_y_center_t)))/(norm((l_x_center_t,l_y_center_t))**2)
    koordinate_y_t = (skalarprodukt(bild_zentrum_t,(b_x_center_t,b_y_center_t)))/(norm((b_x_center_t,b_y_center_t))**2)
    k_position_vermutung_x = 2.33 + koordinate_x_t * 1.24
    k_position_vermutung_y = 1.15 + koordinate_y_t * 1.85
    vermutung = (k_position_vermutung_x,k_position_vermutung_y)

  else:
    koordinaten_ursprung = (b_x_center,b_y_center)
    bild_zentrum_t = (bild_zentrum[0] - koordinaten_ursprung[0],bild_zentrum[1] - koordinaten_ursprung[1])
    gr_x_center_t = gr_x_center - koordinaten_ursprung[0]
    gr_y_center_t = gr_y_center - koordinaten_ursprung[1]
    r_x_center_t = r_x_center - koordinaten_ursprung[0]
    r_y_center_t = r_y_center - koordinaten_ursprung[1]
    koordinate_x_t = (skalarprodukt(bild_zentrum_t,(gr_x_center_t,gr_y_center_t)))/(norm((gr_x_center_t,gr_y_center_t))**2)
    koordinate_y_t = (skalarprodukt(bild_zentrum_t,(r_x_center_t,r_y_center_t)))/(norm((r_x_center_t,r_y_center_t))**2)
    k_position_vermutung_x = 3.57 - koordinate_x_t * 1.24
    k_position_vermutung_y = 1.15 + koordinate_y_t * 1.85
    vermutung = (k_position_vermutung_x,k_position_vermutung_y)


  interessanter_index = 0
  gesuchtes_min = abstand_berechnen(liste_aller_schnittpunkte[0],vermutung)
  for i in range (len(liste_aller_schnittpunkte)):
    if abstand_berechnen(liste_aller_schnittpunkte[i],vermutung) < gesuchtes_min:
      interessanter_index = i
      gesuchtes_min = abstand_berechnen(liste_aller_schnittpunkte[i],vermutung)
  aktuelle_position = liste_aller_schnittpunkte[interessanter_index]
  #print("LOKALISIERUNG: ", aktuelle_position)
  gps_pub = rospy.Publisher("/visual_gps",PointStamped)
  message = PointStamped()
  message.header.stamp = rospy.get_rostime()
  message.header.frame_id = "map"
  message.point.x = liste_aller_schnittpunkte[interessanter_index][0]
  message.point.y = liste_aller_schnittpunkte[interessanter_index][1]
  #TODO publish rausnehmen?
  gps_pub.publish(message)

  #return(liste_aller_schnittpunkte[interessanter_index])

  #plt.plot([gr_x_center,b_x_center,l_x_center,r_x_center,240],[gr_y_center,b_y_center,l_y_center,r_y_center, 320], 'ro')
  #plt.axis([0,480,0,640])
  #plt.show()

  ###################GPS ENDE####################
  ##################RACING######################

  #point_1_1_1 
  p1 = (1.94,0.251)
  #1.1.11
  p1_2 = (2.45,0.275)
  p2 = (2.95,0.3)
  p2_2 = (3.5,0.32)
  #point_1_1_22
  #4.05 
  p3 = (4.05,0.35)
  p3_2 = (4.35,0.425)
  #point_1_1_29 
  p4 = (4.77,0.5)
  #1.1.33
  p4_2 = (4.95,0.5)
  p5 = (5.15,0.5)
  p5_2 = (5.35,0.8)
  #point_1_1_44
  p6 = (5.56,1.15)
  p6_2 = (5.58,1.72)
  #1.1.56 
  p7=(5.6,2.33)
  p7_2 = (5.5,2.83)
  #bis hierhin in die mitte verrueckt
  #point_1_1_67 
  p8 = (5.4,3.37)
  p8_2 = (5.1,3.62)
   #point_1_1_77 
  p9 = (4.83,3.86)
  p9_2 = (4.4,3.8)
  #point_1_1_85 
  p10 = (4.05,3.73)
  p10_2 = (3.75,3.6)
  #1.1.92
  p11 = (3.50,3.53)
  p11_2 = (3.25,3.5)
  #point_1_1_99 
  p12 = (2.95,3.48)
  p12_2 = (2.55,3.6)
  #1.1.104
  p13 = (2.22,3.81)
  p13_2 = (2,3.83)
  #point_1_1_109 
  p14 = (1.73,3.85)
  p14_2 = (1.3,3.6)
  #1.1.118
  p15 = (0.934,3.46)
  p15_2 = (1,3.5)
  #point_1_1_116 
  p16 = (1.08,3.59)
  p16_2 = (0.75,3)
  #point_1_1_128 
  p17 = (0.529,2.58)
  #1.1.138
  p18=(0.59,1.57)
  #point_1_1_148 
  p19 = (0.956,0.662)

  #den "1." punkt setzen wir hinten an, um wieder dort zu landen nach einer runde
  punkte_array.append(p2)
  #punkte_array.append(p2_2)
  punkte_array.append(p3)
  #punkte_array.append(p3_2)
  punkte_array.append(p4)
  #punkte_array.append(p4_2)
  punkte_array.append(p5)
  #punkte_array.append(p5_2)
  punkte_array.append(p6)
  #punkte_array.append(p6_2)
  punkte_array.append(p7)
  punkte_array.append(p7_2)
  punkte_array.append(p8)
  punkte_array.append(p8_2)
  punkte_array.append(p9)
  punkte_array.append(p9_2)
  punkte_array.append(p10)
  punkte_array.append(p10_2)
  punkte_array.append(p11)
  punkte_array.append(p11_2)
  punkte_array.append(p12)
  punkte_array.append(p12_2)
  punkte_array.append(p13)
  punkte_array.append(p13_2)
  punkte_array.append(p14)
  punkte_array.append(p14_2)
  punkte_array.append(p15)
  punkte_array.append(p15_2)
  punkte_array.append(p16)
  punkte_array.append(p16_2)
  punkte_array.append(p17)
  punkte_array.append(p18)
  punkte_array.append(p19)
  punkte_array.append(p1)

  gps_printen +=1

  if gps_printen % 20 == 0:
    print(aktuelle_position)
    gps_printen = 0
    print("AKTUELLER YAW: ",aktueller_yaw)

  naechstes_ziel = punkte_array[(punkte_array_index)%len(punkte_array)]
  naechstes_ziel_plus_eins = punkte_array[(punkte_array_index+1)%len(punkte_array)]
  naechstes_ziel_plus_zwei = punkte_array[(punkte_array_index+2)%len(punkte_array)]
  naechstes_ziel_plus_drei = punkte_array[(punkte_array_index+3)%len(punkte_array)]
  naechstes_ziel_plus_vier = punkte_array[(punkte_array_index+4)%len(punkte_array)]
  naechstes_ziel_plus_fuenf = punkte_array[(punkte_array_index+5)%len(punkte_array)]
  naechstes_ziel_plus_sechs = punkte_array[(punkte_array_index+6)%len(punkte_array)]


  #TODO if abstand zum punkt < 0.2 -> punkte_array_index +=1
  #print("kommen wir hier rein?")
  toleranz_abstand = 0.35

  if abstand_berechnen(aktuelle_position,naechstes_ziel) < toleranz_abstand:
    steuerung(aktuelle_position,naechstes_ziel)
  
  elif abstand_berechnen(aktuelle_position,naechstes_ziel_plus_eins) < toleranz_abstand:
    steuerung(aktuelle_position,naechstes_ziel_plus_eins)
  
  elif abstand_berechnen(aktuelle_position,naechstes_ziel_plus_zwei) < toleranz_abstand:
    steuerung(aktuelle_position,naechstes_ziel_plus_zwei)
  
  elif abstand_berechnen(aktuelle_position,naechstes_ziel_plus_drei) < toleranz_abstand:
    steuerung(aktuelle_position,naechstes_ziel_plus_drei)
 
  elif abstand_berechnen(aktuelle_position,naechstes_ziel_plus_vier) < toleranz_abstand:
    steuerung(aktuelle_position,naechstes_ziel_plus_vier)
  
  elif abstand_berechnen(aktuelle_position,naechstes_ziel_plus_fuenf) < toleranz_abstand:
    steuerung(aktuelle_position,naechstes_ziel_plus_fuenf)
  
  elif abstand_berechnen(aktuelle_position,naechstes_ziel_plus_sechs) < toleranz_abstand:
    steuerung(aktuelle_position,naechstes_ziel_plus_sechs)


def steuerung(aktuelle_position,naechstes_ziel):
  global punkte_array_index, punkte_array, aktuelles_steering, aktueller_yaw
  punkte_array_index = (punkte_array_index+1) % len(punkte_array)
  naechstes_ziel = punkte_array[punkte_array_index]
  naechster_verbindungsvektor = verbindungs_vektor_berechnen(aktuelle_position,naechstes_ziel)
  
  ### todo: koennte schief gehen, wenn wir andersrum fahren
  if naechster_verbindungsvektor[0] < 0 and naechster_verbindungsvektor[1] < 0:
    positions_winkel = 360 - get_positions_winkel(naechster_verbindungsvektor)
  else:
    positions_winkel = get_positions_winkel(naechster_verbindungsvektor)
  
  benoetigte_korrektur = (positions_winkel - aktueller_yaw)
    
  if aktuelles_steering > 130:
    steering_pub.publish(min(120,aktuelles_steering + benoetigte_korrektur))
    aktuelles_steering = min(120,aktuelles_steering + benoetigte_korrektur)
    
  elif (aktuelles_steering + benoetigte_korrektur) < 30:
    steering_pub.publish(30)
    aktuelles_steering = 30
  else:
    steering_pub.publish(min(150,aktuelles_steering +benoetigte_korrektur))
    aktuelles_steering = min(150,aktuelles_steering +benoetigte_korrektur)

  if aktuelles_steering < 30:
    steering_pub.publish(30)
    aktuelles_steering = 30

  for i in range(2):
    print("STEERING: ",aktuelles_steering)  
  for i in range(2):
    print("AKTUELLER YAW: ",aktueller_yaw)
  for i in range(2):
    print("POSITIONSWINKEL: ",positions_winkel)
  for i in range(2):
      print("PUNKT ERREICHT, naechster index:", punkte_array_index)

def get_positions_winkel(naechster_verbindungsvektor):
  return math.degrees(math.acos(skalarprodukt(naechster_verbindungsvektor,(1,0)) / norm	(naechster_verbindungsvektor)))

def callback_yaw(data):
  global aktueller_yaw, start_yaw, einmal_ausgefuehrt
  if not einmal_ausgefuehrt:
    start_yaw = data.data
    einmal_ausgefuehrt = True
  aktueller_yaw = data.data - start_yaw

def callback_steering(data):
  global aktuelles_steering
  aktuelles_steering = data.data

def verbindungs_vektor_berechnen(aktuelle_position,ziel_position):
  diff_x = ziel_position[0] - aktuelle_position[0]
  diff_y = ziel_position[1] - aktuelle_position[1]
  return(diff_x,diff_y)

def abstand_berechnen(punkt1,punkt2):
  diff_x = punkt1[0] - punkt2[0]
  diff_y = punkt1[1] - punkt2[1]
  return norm((diff_x,diff_y)) 

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
  if not(schnittpunkte_der_kreise((punkt1,radius),(punkt2,radius)) == None):
    mittelpunkt_1, mittelpunkt_2 = schnittpunkte_der_kreise((punkt1,radius),(punkt2,radius))
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

rospy.sleep(0.5)
speed_pub = rospy.Publisher("/manual_control/speed",Int16, latch = True)
rospy.sleep(0.5)
stop_start_pub = rospy.Publisher("/manual_control/stop_start",Int16,latch = True)
rospy.sleep(0.5)
steering_pub = rospy.Publisher("/manual_control/steering",Int16, latch = True)
rospy.sleep(1)
stop_start_pub.publish(0)
rospy.sleep(1.5)
steering_pub.publish(80)
rospy.sleep(0.5)
speed_pub.publish(-400)
ransac_sub = rospy.Subscriber("/usb_cam/image_raw",Image,callback, queue_size=1)
yaw_sub = rospy.Subscriber("/model_car/yaw",Float32,callback_yaw, queue_size=1)
steering_sub = rospy.Subscriber("/manual_control/steering",Int16,callback_steering, queue_size=1)

rospy.spin()

