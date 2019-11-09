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


target_list = np.array([np.array([28.75414290238, 77.11490918927977 ]), np.array([28.752164395389574, 77.11208826546536]),
    np.array([28.752659021986783, 77.11357566113641 ]), np.array([28.751939564869787, 77.11270373999307]), np.array([28.750950310777796, 77.11439629384002]), 
    np.array([28.75346841118562, 77.11485790084004]), np.array([28.750995277183534, 77.11270373999307]), np.array([28.753243580784122, 77.11485790084004]), 
    np.array([28.750815412960932, 77.11219084455331]),np.array([28.751804666510544, 77.11331921452077 ]), np.array([28.75328854717011, 77.11326792387256]),np.array([28.74969126087462, 77.11260116090511]),
    np.array([28.7533784793307, 77.11326792387256]),np.array([28.751759700430252, 77.11331921452077]), np.array([28.751714734349957, 77.11331921452077 ]),np.array([28.749736226954923, 77.11260116090511]), 
    np.array([28.752209361213502, 77.11342179360872]),np.array([28.753333513250407, 77.11326792387256]), np.array([28.75068051469051, 77.112344712081]),np.array([28.750860379021468, 77.11229342364126]), 
    np.array([28.752164395133203, 77.11342179360872]), np.array([28.7522543272938, 77.11342179360872]), np.array([28.7522992933741, 77.11342179360872]),np.array([28.752344259454404, 77.11342179360872 ]), 
    np.array([28.752344259543115 ,77.1129601866087]), np.array([28.75229929346282, 77.1129601866087]), np.array([28.752254327382527, 77.1129601866087])])



def colab_search(UAVs, N, M, area_cell, g_list):
	"""
	UAVs: list of UAV objects
	N: Total Number of UAVs
	M: total number of cell centres
	"""

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

		# some basic initializations
		for UAV in UAVs:
			UAVdict[UAV.UAVID] = np.zeros(M)
			UAV.updatelocation()
			UAV.update_density()


		for UAV in UAVs:

			for UAV2 in UAVs:
				UAV2.updatelocation()
				UAV.turn()


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
			Z_klist = takeobservations(70, 30, UAV.getlocation()[0], UAV.getlocation()[1], UAV.heading(), 10, 10, target_list, UAV.g_list, M)
			for i in range(M):
				if Z_klist[i] == 1:
					UAV.wplist.append(g_list[i])
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
			UAV.update_velocity(100000)


			time.sleep(0.01)

	for UAV in UAVs:
		print("UAV: ", UAV.getlocation())





