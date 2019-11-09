from scipy.spatial import Voronoi
import numpy as np 
import math
import time
import sys
from GraphClass import Graph
import itertools
from Swarm_Class import UAVSwarmBot
import dronekit
from dividesearcharea import coordinates, takeobservations, rectangle_mid_point
from uncertainity_functions import average_uncertainity, contains_point
from main import colab_search
from generate_voronoi import voronoi


m = 1000
n = 1000


centre_wp = [28.754143,77.115781] # enter coordinate


# make uav list and arm and take off
# should return a list of centre coordinates
target_list = np.array([np.array([28.75414290238, 77.11490918927977 ]), np.array([28.752164395389574, 77.11208826546536]),
    np.array([28.752659021986783, 77.11357566113641 ]), np.array([28.751939564869787, 77.11270373999307]), np.array([28.750950310777796, 77.11439629384002]), 
    np.array([28.75346841118562, 77.11485790084004]), np.array([28.750995277183534, 77.11270373999307]), np.array([28.753243580784122, 77.11485790084004]), 
    np.array([28.750815412960932, 77.11219084455331]),np.array([28.751804666510544, 77.11331921452077 ]), np.array([28.75328854717011, 77.11326792387256]),np.array([28.74969126087462, 77.11260116090511]),
    np.array([28.7533784793307, 77.11326792387256]),np.array([28.751759700430252, 77.11331921452077]), np.array([28.751714734349957, 77.11331921452077 ]),np.array([28.749736226954923, 77.11260116090511]), 
    np.array([28.752209361213502, 77.11342179360872]),np.array([28.753333513250407, 77.11326792387256]), np.array([28.75068051469051, 77.112344712081]),np.array([28.750860379021468, 77.11229342364126]), 
    np.array([28.752164395133203, 77.11342179360872]), np.array([28.7522543272938, 77.11342179360872]), np.array([28.7522992933741, 77.11342179360872]),np.array([28.752344259454404, 77.11342179360872 ]), 
    np.array([28.752344259543115 ,77.1129601866087]), np.array([28.75229929346282, 77.1129601866087]), np.array([28.752254327382527, 77.1129601866087])])

p = rectangle_mid_point(1000, 1000, 28.754143,77.115781, 0)
cell_list = coordinates(m, n, p[0][0], p[0][1], 10, 10, 0)
vehicle = list()
for i in range(20):
    vehicle.append(UAVSwarmBot("127.0.0.1:" + str(14550 + i*10)))
    vehicle[i].g_list = cell_list
    vehicle[i].UAVID += i

"""
point0 = dronekit.LocationGlobalRelative(28.753493,77.111159, 35)
point1 = dronekit.LocationGlobalRelative(28.753493, 77.112158, 35)
point2 = dronekit.LocationGlobalRelative(28.753493,77.113183, 35)
point3 = dronekit.LocationGlobalRelative(28.753481,77.114258, 35)
point4 = dronekit.LocationGlobalRelative(28.753470 ,77.115245, 35)
points = [point0, point1, point2, point3, point4]
"""

for i in range(20):
    vehicle[i].arm_and_takeoff(5)
    vehicle[i].vehicle.simple_goto(points[i])

time.sleep(120)

colab_search(vehicle, 20, 10000, 100, cell_list)