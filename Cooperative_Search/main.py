from scipy.spatial import Voronoi, votronoi_plot_2d
import numpy as np 
import scipy.integrate as integrate
import scipy.special as special
import math
import geopy
import pandas as pd 
import socket
import json
import time
import sys
from GraphClass.py import Graph
import itertools
from Swarm_Class.py import UAVSwarmBot
from dividesearcharea.py import voronoi, coordinates, takeobservations
from uncertainity_functions.py import average_uncertainity, contains_point

m = 1000
n = 1000
centre_wp = [] # enter coordinate


# make uav list and arm and take off
# should return a list of centre coordinates

target_list = []

vehicle = list()
for i in range(N):
    vehicle.append(UAVSwarmBot("127.0.0.1:" + str(14550 + i*10)))
    p = rectangle(m, n, centre_wp[0], centre_wp[1], vehicle[i].heading)
	g_list = coordinates(m, n, p[0][0], p[0][1], 10, 10, vehicle[i].heading)
	vehicle[i].g_list = g_list

for i in range(N):
    vehicle[i].arm_and_takeoff(10)


def colab_search(UAVs, N, M, density_list, average_uncertainity_deviation, area_cell, g_list):
	"""
	UAVs: list of UAV objects
	N: Total Number of UAVs
	M: total number of cell centres
	"""
	while abs(average_uncertainity(N, M, density_list, avg_uncertainity_deviation)) <= 0.1:
		# create partition for each UAV
		# update_Q for each UAV
		# transmit_Q for each UAV
		# update_fusion for each UAV
		# calculate A for each UAV
		# using A calculate CM for each UAV
		# generate control law for each UAV
		# time.sleep(1)
		friend_points = []
		# contains the Z_list for each UAV at each iteration
		UAVdict = dict()     
		for UAV in UAVs:
			# getneighbors will return adjecency matrix of the graph class
			neighbormatrix = UAV.getneighbors()[UAV.UAVID]
			for friend in neighbormatrix:
				if friend == 1:
					friend_points.append(friend.getlocation())
			UAV.updatevoronoi(friend_points)

			# take observations and append in Z_klist
			# Z_klist = []
			Z_klist = takeobservation(70, 30,UAV.getlocation()[0], UAV.getlocation()[1], UAV.heading(), 10, 10, target_list, UAV.g_list)

			# update self Q
			UAV.update_Q(Z_klist) 
			# transmit updated Q based on self measurement
			# transmit and recieve on different threads
			# for now instead of transmitting add to the dictionary
			UAVdict[UAV.UAVID] = Z_klist
			# for keys in dict get a multidimenstional list of friend data
			# friend_Q_list = UAV.recievedata()
			for key in UAVdict.keys():
				friend_Q_list.append(UAVdict[key])

			# fusion update
			UAV.fusion_update(friend_Q_list)

			# calculate A
			UAV.update_A(area_cell, g_list, M)

			# calculate CM
			UAV.update_CM(area_cell, g_list)

			time.sleep(0.1)

			# update velocity using control law
			UAV.update_velocity(1)

