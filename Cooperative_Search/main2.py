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

centre_wp = [28.751700,77.113249] # enter coordinate


# make uav list and arm and take off
# should return a list of centre coordinates

target_list = [[28.74967771793720,77.11411783334879], [28.74967771793748,77.11411783334871],[28.751431394862347,77.11519490731644],[28.75143139467584,77.11616940070783],[28.751836089398484,77.11616940070783]]

vehicle = list()
for i in range(5):
    vehicle.append(UAVSwarmBot("127.0.0.1:" + str(14550 + i*10)))
    p = rectangle_mid_point(m, n, centre_wp[0], centre_wp[1], vehicle[i].heading())
    g_list = coordinates(m, n, p[0][0], p[0][1], 10, 10, vehicle[i].heading())
    vehicle[i].g_list = g_list
    vehicle[i].UAVID += i


point0 = dronekit.LocationGlobalRelative(28.754168, 77.110976, 35)
point1 = dronekit.LocationGlobalRelative(28.754180, 77.111711, 35)
point2 = dronekit.LocationGlobalRelative(28.754180, 77.112815, 35)
point3 = dronekit.LocationGlobalRelative(28.754180, 77.113616, 35)
point4 = dronekit.LocationGlobalRelative(28.754168, 77.115033, 35)
points = [point0, point1, point2, point3, point4]

for i in range(5):
    vehicle[i].arm_and_takeoff(5)
    vehicle[i].vehicle.simple_goto(points[i])

colab_search(vehicle, 5, 10000, 100, g_list)