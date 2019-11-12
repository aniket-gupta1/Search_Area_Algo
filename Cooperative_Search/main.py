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

target_list = np.array([np.array([28.755413308079383, 77.1133998428576]), np.array([28.756762289717177, 77.11545145223369]),
np.array([28.754603915780642, 77.12099079754887]), np.array([28.75460391948231, 77.1111430725439]),np.array([28.752355612035693, 77.12027272985]), np.array([28.752895205307713, 77.11945208609963]),
np.array([28.748848260202106, 77.11381016473283 ]), np.array([28.750556971253307, 77.11381016473283]),np.array([28.75037710450266, 77.12027272985]), np.array([28.751905952466625, 77.11699015484842]),
np.array([28.75316500283068, 77.11668241785934]), np.array([28.757122019092186, 77.11350241890909]),np.array([28.753704597143887, 77.11309209703388]), np.array([28.75568310348152, 77.11627209598412]),
np.array([28.75073683449474, 77.11668241785934]), np.array([28.75001737613025, 77.11955467098574]),np.array([28.756402561845857, 77.1133998428576]), np.array([28.75505357739327, 77.11883660328687]),
np.array([28.754334122807787, 77.11165597047062]), np.array([28.751995884395786, 77.11760563766124]),np.array([28.75208581601658, 77.11904176422445]), np.array([28.75757167916251, 77.11545145223369]),
np.array([28.750916697967458, 77.11893918817296]), np.array([28.75352473147304, 77.11668241785934]),np.array([28.753075069166247, 77.12068305172518]), np.array([28.756492492001193, 77.11873402723538]),
np.array([28.757122019786184, 77.11165597047062]), np.array([28.756582426012844, 77.11381016473283]),np.array([28.75631262983952, 77.1129895209824]), np.array([28.751546224788488, 77.11442564754564]),
np.array([28.75640255960931, 77.11934951004814]), np.array([28.75334486688188, 77.11740047672363 ]),np.array([28.75136636027445, 77.11493854547234]), np.array([28.757301884300027, 77.1111430725439]),
np.array([28.755862966722887, 77.11914434911056 ]), np.array([28.75316500148104, 77.12027272985]),np.array([28.7539743939341, 77.11227145328346]), np.array([28.754513985278184, 77.11657983297322]),
np.array([28.75118649741841, 77.11104048765779]), np.array([28.757122017973817, 77.11647725692173])])



def colab_search(UAVs, N, M, area_cell, g_list):
	"""
	UAVs: list of UAV objects
	N: Total Number of UAVs
	M: total number of cell centres
	"""
	print("search started")


	while True:
		# create partition for each UAV
		# update_Q for each UAV
		# transmit_Q for each UAV
		# update_fusion for each UAV
		# calculate A for each UAV
		# using A calculate CM for each UAV
		# generate control law for each UAV
		# time.sleep(1)
		
		if average_uncertainity(N,M,UAVs) <= 0.3:
			break
		

		friend_points = np.zeros((N,2))
		UAVdict = dict()
		# contains the Z_list for each UAV at each iteration

		# some basic initializations
		for UAV in UAVs:
			UAVdict[UAV.UAVID] = np.zeros(M)
		
		for UAV in UAVs:

			# getneighbors will return adjecency matrix of the graph class
			t = time.time()
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
			UAV.update_A(area_cell, M)


			# calculate CM

			UAV.update_CM(area_cell, M)



			# update velocity using control law
			UAV.update_velocity(1000)
			t2 = time.time()
			print(t2-t)
			print("vel: ", UAV.getvel())
			#print(UAV.getvel())

			time.sleep(0.001)

	for UAV in UAVs:
		print("UAV: ", UAV.getlocation())





