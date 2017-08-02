#!/usr/bin/env python

# --- imports ---
import rospy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from matplotlib import pyplot as plt
from sklearn import linear_model, datasets
import numpy as np

# --- definitions ---
def resetGrid():
    global occupancy_grid
    
    # set all values to "FREE"
    occupancy_grid.data = [0]*(occupancy_grid.info.width*occupancy_grid.info.height)
    

# to a given cartesian x,y coordinate, mark the corresponding cell in the grid as "OCCUPIED"
def setCell(x,y):
    global occupancy_grid

    res = occupancy_grid.info.resolution
    x_scaled = (x * 1.0 / res) + occupancy_grid.info.width/2
    y_scaled = (y * 1.0 / res) + occupancy_grid.info.height/2

    if x_scaled >= occupancy_grid.info.width or x_scaled < 0 or y_scaled >= occupancy_grid.info.height or y_scaled < 0:
        return

    offset = (int(round(x_scaled)) - 1) * occupancy_grid.info.height
    try:
        occupancy_grid.data[int(offset) + int(round(y_scaled) - 1)] = 100
        #occupancy_grid.data[int(round(x_scaled)) + int(round(y_scaled) - 1)] = 100
    except IndexError as e:
	print(len(occupancy_grid.data))
	print(int(round(x_scaled)) + int(round(y_scaled) - 1))
        




def scanCallback(scan_msg):
    global occupancy_grid

    resetGrid()

    # convert scan measurements into an occupancy grid
    constant = 0.0174532923847

    #x und y werte werden in seperaten arrays gespeichert
    x_array = [] 
    y_array = []

    for a in range(0,len(scan_msg.ranges)):
        #wir kriegen etwas zurueck -> hier ist was
        if scan_msg.ranges[a] >= 0.15 and scan_msg.ranges[a] <= 6:
            y=np.sin(a*constant) * scan_msg.ranges[a]
            x=np.cos(a*constant) * scan_msg.ranges[a]
            #zelle markieren
            setCell(x,y)
            x_array.append(x)
            y_array.append(y)


    #zweites_x_array = [1,2]
    #x_2 = np.array(zweites_x_array)


    #pub_grid.publish(occupancy_grid)
    #ransac:
    model_ransac = linear_model.RANSACRegressor(linear_model.LinearRegression())
    
    #print("Laengen:",len(x_array),len(y_array))

    model_ransac.fit(np.array(x_array).reshape(len(x_array),1),np.array(y_array).reshape(len(y_array),1))

    #line_y_ransac = model_ransac.predict(x_2)

    steigung = model_ransac.estimator_.coef_
    print(steigung)
    return (model_ransac.estimator_.coef_)

    #print(x_2)
    #return line_y_ransac

# --- main ---
rospy.init_node("scan_grid")

# init occupancy grid
occupancy_grid = OccupancyGrid()

occupancy_grid.header.frame_id = "laser"
occupancy_grid.info.resolution = 0.1 # in m/cell

# width x height cells
occupancy_grid.info.width = 100
occupancy_grid.info.height = 100

# origin is shifted at half of cell size * resolution
occupancy_grid.info.origin.position.x = int(-1.0 * occupancy_grid.info.width / 2.0) * occupancy_grid.info.resolution
occupancy_grid.info.origin.position.y = int(-1.0 * occupancy_grid.info.height / 2.0) * occupancy_grid.info.resolution
occupancy_grid.info.origin.position.z = 0
occupancy_grid.info.origin.orientation.x = 0
occupancy_grid.info.origin.orientation.y = 0
occupancy_grid.info.origin.orientation.z = 0
occupancy_grid.info.origin.orientation.w = 1


rospy.Subscriber("scan", LaserScan, scanCallback, queue_size=100)
pub_grid = rospy.Publisher("scan_grid", OccupancyGrid, queue_size=100)

rospy.spin()
