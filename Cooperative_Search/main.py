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
from generate_voronoi import voronoi



target_list = [[28.74967771793720,77.11411783334879], [28.74967771793748,77.11411783334871],[28.751431394862347,77.11519490731644],[28.75143139467584,77.11616940070783],[28.751836089398484,77.11616940070783]]

def colab_search(UAVs, N, M, area_cell, g_list):
	"""
	UAVs: list of UAV objects
	N: Total Number of UAVs
	M: total number of cell centres
	"""
	for UAV in UAVs:
		UAV.updatelocation()
	while True:
		# create partition for each UAV
		# update_Q for each UAV
		# transmit_Q for each UAV
		# update_fusion for each UAV
		# calculate A for each UAV
		# using A calculate CM for each UAV
		# generate control law for each UAV
		# time.sleep(1)
		if average_uncertainity(N,M,UAVs) <= 0.1:
			break

		friend_points = np.zeros((N,2))
		UAVdict = dict()
		# contains the Z_list for each UAV at each iteration
		for UAV in UAVs:
			UAVdict[UAV.UAVID] = np.zeros(M)

		for UAV in UAVs:
			
			for UAV in UAVs:
				UAV.updatelocation()

			# getneighbors will return adjecency matrix of the graph class
			for friends in UAVs:
				UAV.connect_to_neighbor(friends)

			neighbormatrix = UAV.getneighbors()[UAV.UAVID]
			for i in range(len(neighbormatrix)):
				if neighbormatrix[i] == 1:
					friend_points[i] = (UAVs[i].getlocation())
			UAV.updatevoronoi(friend_points)

			# take observations and append in Z_klist
			# Z_klist = []
			Z_klist = takeobservations(70, 30,UAV.getlocation()[0], UAV.getlocation()[1], UAV.heading(), 10, 10, target_list, UAV.g_list, M)
			# update self Q

			UAV.update_Q(Z_klist)
			# transmit updated Q based on self measurement
			# transmit and recieve on different threads
			# for now instead of transmitting add to the dictionary
			UAVdict[UAV.UAVID] = Z_klist
			# for keys in dict get a multidimenstional list of friend data
			# friend_Q_list = UAV.recievedata()
			friend_Q_list = []
			for key in UAVdict.keys():
				friend_Q_list.append(UAVdict[key])

			# fusion update
			UAV.fusion_update(friend_Q_list)

			# density update
			UAV.update_density()

			# calculate A
			UAV.update_A(area_cell, g_list, M)

			# calculate CM
			UAV.update_CM(area_cell, g_list, M)

			# update velocity using control law
			UAV.update_velocity(1)
			time.sleep(0.1)

	for UAV in UAVs:
		print("UAV: ", UAV.getlocation())





