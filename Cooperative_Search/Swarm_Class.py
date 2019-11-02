from scipy.spatial import Voronoi, votronoi_plot_2d
import numpy as np 
import dronekit
import math
import geopy
import pandas as pd 
import socket
import json
import time
import sys
from GraphClass.py import Graph
import itertools
from pymavlink import mavutil
from uncertainity_functions.py import contains_point

class UAVSwarmBot:
	UAVID = itertools.count().next
	def __init__(self, s, network_range = 20, num_uavs = 5, p=0.9, q=0.3):
		"""
		 location: a list of x and y coordinate of UAV initially, converted to a position vector (numpy)
		 Q: list/hashmap of P transforms of all cells in the region
		 network_range: distance between 2 UAV which ensures proper data transfer, edge length of undirected graph
		 n: number of vertices of the undirected graph/ number of UAVs
		 velocity: velocity vector
		 dij = wieght matrix, initialized to matrix of zeros
		 p: probability of presence of target hashmap
		 q: probability of absence of a target hashmap
		 CM: centroid of voronoid region vector
		 density: list of densities of each cell
		"""
		self.vehicle = dronekit.connect(s)
		self.g_list = []
		self.p = p 
		self.q = q
		self.__Q = np.zeros(10000)
		self.__network_range = network_range
		self.num_uavs = num_uavs
		self.__neighbors = Graph(n-1) # or just use list for simplicity
		self.__velocity = np.zeros(2)
		self.__dij = np.zeros(num_uavs, num_uavs)
		self.Kn = 2
		self.Ku = 1
		self.__voronoi_part_map = []
		self.density = []
		self.A = 0 
		self.CM = []
		self.wplist=[]
		self.id=int(self.vehicle.parameters['SYSID_THISMAV'])
		self.UAVID = resource_cl.newid() # change for decentralized


	def __str__(self):
		return "UAVSwarmBot is the decentralized Probability Map based control law generation for an individual UAV."

	def getQ(self):
		"""	
		returns: list Q with P tranforms of each cell as per the UAV object
		"""
		return self.__Q

	def getlocation(self):
		"""
		returns: current location of UAV
		"""
		location = np.array([self.vehicle.location.global_frame.lat,self.vehicle.location.global_frame.lon])
		return location


	def getrange(self):
		"""
		returns: network range required for communication between any two UAVs
		"""
		return self.__network_range

	def getneighbors(self):
		"""
		returns: string of the adjecency matrix of neighbors of the UAV
		"""

		return self.__neighbors.adjmatrix

	def getvoro(self):
		"""
		returns voronoi point list
		"""
		return self.__voronoi_part_map

	def update_density(self):
		self.density = []
		self.density.append(exp(-self.Kn*np.linalg.norm(self.__Q)))
		
	def update_Q(self, Zk_list):
		"""
		changes the value of Q list
		"""

		for i in range(len(self.__Q)):
			if Zk_list[i] == 1:
				v = math.log(self.q/self.p)
			elif Zk_list[i] == 0:
				v = math.log((1- self.q)/(1-self.p))
			else:
				v = 0

			self.__Q[i] += v

	def fusion_update(self, friend_Q_list):
		"""
		"""
		for j in range(self.num_uavs):
			if self.__neighbors.containsEdge(self.UAVID, j):
				for i in range(len(self.__Q)):
					self.__Q[i] = self.__dij[self.UAVID][j]*friend_Q_list[j][i]

	def update_A(self, area_cell, g_list, M):
	"""
	g_list: list of coordinates of centre of all points
	"""
		g_list = np.array(g_list[i])
		self.A = 0
		for i in range(M):
			if contains_point(g_list[i], self.__voronoi_part_map):
				self.A += area_cell*self.density[i]

	def update_CM(self, area_cell, g_list):
		self.CM = []
		g_list = np.array(g_list)
		for i in range(M):
			if contains_point(g_list[i], self.__voronoi_part_map):
				self.CM[0]+=((1/self.A)*(area_cell)*(self.density[i]))*g_list[i][0]
				self.CM[1]+=((1/self.A)*(area_cell)*(self.density[i]))*g_list[i][1]

	def update_velocity(self, Ku):
		"""
		changes the velocity vector of the UAV
		"""
		vel_x = -Ku*(self.CM[0] - self.__location[0])
		vel_y = -Ku*(self.CM[1] - self.__location[1])
		new_vel = [vel_x, vel_y]
		mynorm = np.linalg.norm(new_vel)

		if mynorm > 5:
			new_vel[0] = new_vel[0]*5/mynorm
			new_vel[1] = new_vel[1]*5/mynorm
		self.__velocity = new_vel

		"""
        Move vehicle in direction based on specified velocity vectors.
        """
		msg = self.vehicle.message_factory.set_position_target_global_int_encode(
		    0,       # time_boot_ms (not used)
		    0, 0,    # target system, target component
		    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
		    0b0000111111000111, # type_mask (only speeds enabled)
		    0, # lat_int - X Position in WGS84 frame in 1e7 * meters
		    0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
		    0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
		    # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
		    self.__velocity[0], # X velocity in NED frame in m/s
		    self.__velocity[1], # Y velocity in NED frame in m/s
		    0, # Z velocity in NED frame in m/s
		    0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
		    0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

		self.vehicle.send_mavlink(msg)


	def add_voro_point(self, pt):
		"""
		add a voronoi partition coordinate
		"""
		self.__voronoi_part_map.append(pt)

	def update_location(self, newlocation):
		"""
		changes the current location of the UAV
		"""
		points = dronekit.LocationGlobalRelative(new_location[0], newlocation[1], self.altitude)
		self.vehicle.simple_goto(points)

	def updatevoronoi(self, point_list):
		"""
		"""
		self.__voronoi_part_map = point_list

	def norm(self, target_coordinate):
		"""
		target_coordinate: a list of 2 elements containing x and y coordinate of target respectively
		returns float: norm of the eucledian distance vector rounded to 4 decimal places
		"""

		# initialize zero matrix with 2 elements 
		temp = np.zeros(2)

		current_x = self.__location[0]
		current_y = self.__location[1]

		# converting the target_coordinate list in a numpy vector
		target_coordinate = np.array(target_coordinate).transpose()

		# add the x and y coordinate difference in the temp array respectively and calculate norm of the same
		temp[0] = target_coordinate[0] - current_x
		temp[1] = target_coordinate[1] - current_y
		return np.linalg.norm(temp).round(4)

	def inrange(self, target_coordinate):
		"""
		target_coordinate: a list of 2 elements containing x and y coordinate of target respectively
		returns bool: checks if the given coordinate is in communication range with the UAVSwarmBot
		"""
		return  self.norm(target_coordinate) <= self.__network_range

	def connect_to_neighbor(self, friend):
		"""
		friend: Numpy vector of friend ID
		adds edge between 2 UAVs
		"""
		if self.inrange(friend.getlocation):
			self.__neighbors.addEdge(self.UAVID, friend.UAVID)

	def remove_neighbor(self, friend):
		"""
		friend: Numpy vector of friend ID
		deletes edge between 2 UAVs
		"""
		if not self.inrange(friend.getlocation):
			self.__neighbors.removeEdge(self.UAVID, friend.UAVID)

	def construct_weight_matrix(self):
		"""
		numneighbors: number of neighbors of the uav at the particular instant
		constructs a weight matrix dij initialized to 0
		"""
		count = 0
		for i in range(self.num_uavs):
			if self.__neighbors.containsEdge(self.UAVID, i):
				count += 1
		numneighbors = count

		for i in range(self.num_uavs):
			for j in range(self.num_uavs):

				if self.__neighbors.containsEdge(self.UAVID, j):
					if i != j:
						self.__dij[i][j] = 1 - (numneighbors - 1)/self.num_uavs
					else:
						self.__dij[i][j] = 1/self.num_uavs


    def altitude(self):
        return self.vehicle.location.global_relative_frame.alt


    def arm_and_takeoff(self, aTargetAltitude):
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...",self.id)
            time.sleep(1)

            
        print ("Arming motors",self.id)
        
		# Copter should arm in GUIDED mode
		self.vehicle.mode = dronekit.VehicleMode("GUIDED")
		self.vehicle.armed = True

		while not self.vehicle.armed:
		    print (" Waiting for arming...",self.id)
		    time.sleep(1)

		print ("Taking off!",self.id)
		self.vehicle.simple_takeoff(aTargetAltitude)

		while True:
		    print (" Altitude: ", self.vehicle.location.global_relative_frame.alt)      
		    if self.vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.90: 
			print ("Reached target altitude",self.id)
			break
		    time.sleep(1)

	def land(self):
		self.vehicle.mode = dronekit.VehicleMode("LAND")


	def heading(self):
		self.head=self.vehicle.heading
		return self.head

	def transmitdata(self):
		pass

	def recievedata(self):
		pass




