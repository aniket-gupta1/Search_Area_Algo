import dronekit
from pymavlink import mavutil
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
import numpy as np
from math import *
import threading
import random
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from shapely.geometry import LineString
from helper import distance, inside_circle, finalwaypoins, bearing
import sys
import socket

class SwarmBot:

    def __init__(self, s):
        """
        vehicle: dronekit object
        velocity: vector
        s: string sysid
        wplist: list of waypoints
        States- 1: payload present and not dropping
                0: payload present dropping
                -1: payload not present
        """
        self.vehicle = dronekit.connect(s)
        # self.neighbors = []  list of neighbor ids, use the shared memory to access the neighbor data
        self.__velocity= self.vehicle.velocity
        self.__pos= [self.vehicle.location.global_frame.lat,self.vehicle.location.global_frame.lon]
        self.id=int(self.vehicle.parameters['SYSID_THISMAV'])
        print("Connected to vehicle id : " + str(self.id))
        self.__wplist=[]
        self.__payload = 1
        self.__gbestloc = [0, 0]
        self.__bestlocation = [0, 0]
        self.__gbest = 0
        self.__pbest = 0
        self.__droplocation = [0, 0, 0]
        self.__personal_humans = []
        self.__global_humans = []
        self.__futurepayload = []



    def get_pos(self):
        """
        returns: current position of SwarmBot object
        """
        self.__pos= [self.vehicle.location.global_frame.lat,self.vehicle.location.global_frame.lon]
        return self.__pos

    
    def get_vel(self):
        """
        returns: current velocity vector of SwarmBot Object
        """
        return self._velocity

    def get_gbestloc(self):
        return self.__gbestloc

    def get_PersonalHumans(self):
        return self.__personal_humans

    def update_PersonalHumans(self, humans):
        self.__personal_humans.append(humans)

    def get_GlobalHumans(self):
        return self.__global_humans

    def update_GlobalHumans(self, humans):
        self.__global_humans.append(humans)

    def get_droplocation(self):
        return self.__droplocation

    def get_state(self):
        return self.__payload


    def get_bestlocation(self):
        return self.__bestlocation

    def get_gbest(self):
        return self.__gbest

    def get_pbest(self):
        return self.__pbest

    def get_futurepayload(self):
        return self.__futurepayload


    def waypoints(self):
        return self.__wplist


    def heading(self):
        """
        returns: Swarm Objects heading
        """
        return self.vehicle.heading
        

    def altitude(self):
        """
        returns: Swarm Object altitude
        """
        return self.vehicle.location.global_relative_frame.alt


    def update_pos(self,pos,v):
        """
        v: goto velocity
        pos: final position
        """
        # self.position=pos
        pos_x = pos[0]
        pos_y = pos[1]
        pos_z = 5
        self.vehicle.simple_goto(LocationGlobalRelative(pos_x, pos_y, pos_z),v,v)
        self.__pos= [self.vehicle.location.global_frame.lat,self.vehicle.location.global_frame.lon]


    def update_vel(self,v):
        """
        v: new velocity vector
        """
        self.__velocity=v       
        velocity_x=v[0]
        velocity_y=v[1]
        velocity_z=0
        msg = self.vehicle.message_factory.set_position_target_global_int_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, # lat_int - X Position in WGS84 frame in 1e7 * meters
            0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
            0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
            # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
            velocity_x, # X velocity in NED frame in m/s
            velocity_y, # Y velocity in NED frame in m/s
            velocity_z, # Z velocity in NED frame in m/s
            0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        self.vehicle.send_mavlink(msg)


    def update_waypoints(self,p):
        """
        p: list of locations to be added
        """
        self.__wplist=[]
        for element in p:
           self.__wplist.append(element)
        #return self.__wplist


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


    def drop(self):
        """
        update state of payload
        """
        self__payload = -1


    def generate_mopso_velocity(self, timer, finalwaypoints, num, TMAX, wstart, wend, GlobalUavData):

        vel = [0, 0, 0]
        
        R1 = (random.randrange(0,100,10)/100)
        R2 = (random.randrange(0,100,10)/100)
        R3 = (random.randrange(0,100,10)/100)
        w = (wstart - (wstart - wend)*((timer / TMAX)**2))

        if self.__gbestloc[0] and not self.__bestlocation[0]:
            vel[0] = w*(self.__velocity[0]) + R3*1000*(finalwaypoints[self.id-1][0]-self.get_pos()[0]) + R2*1000*(self.__gbestloc[0]-self.get_pos()[0]) 
            vel[2]=0
            vel[1] = w*(self.__velocity[1]) + R3*1000*(finalwaypoints[self.id-1][1]-self.get_pos()[1]) + R2*1000*(self.__gbestloc[1]-self.get_pos()[1]) 

        elif not self.__gbestloc[0] and not self.__bestlocation[0]:
            #print(R3*1000*(finalwaypoints[self.id-1][0]-self.get_pos()[0]),R2*1000*(self.__gbest[0]-self.get_pos()[0]),R1*1000*(self.__bestlocation[1]-self.get_pos()[1]) )
            vel[0] = w*(self.__velocity[0]) + R3*1000*(finalwaypoints[self.id-1][0]-self.get_pos()[0]) + R1*1000*(self.__bestlocation[0]-self.get_pos()[0])
            vel[2]=0
            vel[1] = w*(self.__velocity[1]) + R3*1000*(finalwaypoints[self.id-1][1]-self.get_pos()[1]) + R1*1000*(self.__bestlocation[1]-self.get_pos()[1])

        elif not self.__bestlocation[0] and not self.__gbestloc[0]:
            vel[0] = w*(self.__velocity[0]) + R3*1000*(finalwaypoints[self.id-1][0]-self.get_pos()[0])  
            vel[2]=0
            vel[1] = w*(self.__velocity[1]) + R3*1000*(finalwaypoints[self.id-1][1]-self.get_pos()[1]) 
        else:
            vel[0] = w*(self.__velocity[0]) + R3*1000*(finalwaypoints[self.id-1][0]-self.get_pos()[0]) + R2*1000*(self.__gbest[0]-self.get_pos()[0]) + R1*1000*(self.__bestlocation[0]-self.get_pos()[0])
            vel[2]=0
            vel[1] = w*(self.__velocity[1]) + R3*1000*(finalwaypoints[self.id-1][1]-self.get_pos()[1]) + R2*1000*(self.__gbestloc[1]-self.get_pos()[1]) + R1*1000*(self.__bestlocation[1]-self.get_pos()[1])

        vel = self.inter_uav_collsion(vel, GlobalUavData)

        v_mopso = inside_circle(vel)

        return v_mopso


    def inter_uav_collsion(self, vel, GlobalUavData):
        """
        """
        for friend in GlobalUavData:
            print(self.id,friend)
            if self.id != friend:

                friend_location = GlobalUavData[self.id][str(friend)]['GPS']
                current_location = self.get_pos()

                dist = distance(friend_location[0], friend_location[1], current_location[0], current_location[1])

                if dist < 30:
                    print(dist, self.id, friend)

                if dist < 7:
                    print("inter uav collision aviodance triggered")
                    vel[0] += 10000*(-friend_location[0] + current_location[0])
                    vel[1] += 10000*(-friend_location[1] + current_location[1])
                    vel[2] = 0

                if dist < 4:
                    print("collision")

            return vel


    def checkifpayload(self):

        if (distance(self.__pos[0],self.__pos[1],self.__droplocation[0],self.__droplocation[1]))<=9:
            print("payload dropped")
            self.__payload = -1
            self.__droplocation = [0, 0 ,0]


    # confirm with aman
    def updategbest(self, GlobalUavData, nearestuav): 
        """
        """
        
        g_best = [(self.__gbest, self.__gbestloc)]
        p_best = [(self.__pbest, self.__bestlocation)]    
        newloc = [self.__bestlocation]
        for friend in nearestuav: 
            friend = str(friend)
            g_best.append((GlobalUavData[self.id][friend]["G"], GlobalUavData[self.id][friend]["gbestloc"]))
            p_best.append((GlobalUavData[self.id][friend]["P"], GlobalUavData[self.id][friend]["bestloc"]))

        max_gfriend = max(g_best)
        max_pfriend = max(p_best)
        if max_gfriend < max_pfriend:
            g_best = max_pfriend[0]
            newloc = max_pfriend[1]
        else:
            g_best = max_gfriend[0]
            newloc = max_gfriend[1]

        self.__gbest = g_best
        self.__gbestloc = newloc

    
    def change_gpbest(self, finalwaypoints, p1, p4):
        """
        """
        heading_gbest = bearing(self.get_pos()[0],self.get_pos()[1],self.__gbestloc[0],self.__gbestloc[1])
        heading_bestloc = bearing(self.get_pos()[0],self.get_pos()[1],self.__bestlocation[0],self.__bestlocation[1])
        heading = radians(float(bearing(p1[0],p1[1],p4[0],p4[1])))
        heading_gbest = radians(float(heading_gbest))
        heading_bestloc = radians(float(heading_bestloc))
     
        if degrees(heading_gbest) > degrees(heading + pi/2 + pi/12) or degrees(heading_gbest) < degrees(heading - pi/2 - pi/12):
            # nearest = self.minimumdistance()

            self.__gbest = 0
            self.__gbestloc = [finalwaypoints[self.id-1][0],finalwaypoints[self.id-1][1]]

        if degrees(heading_bestloc) > degrees(heading + pi/2 + pi/12) or degrees(heading_bestloc) < degrees(heading - pi/2 - pi/12):
            self.__pbest = 0
            self.__bestlocation = [finalwaypoints[self.id-1][0],finalwaypoints[self.id-1][1]]


    def minimumdistance(self, GlobalUavData):
        """
        """

        friend_distances = []
        current_position = self.get_pos()
        for friends in GlobalUavData: 
            friends = str(friends)
            friend_location = GlobalUavData[self.id][friends]["GPS"] 
            friend_distances.append((distance(current_position[0],current_position[1],friend_location[0],friend_location[1]), friends))

        friend_distances.sort()
        for element in friend_distances[:5]:
            pso_friends.append(element[1])

        return pso_friends # list of uav ids

    def velocity_post_drop(self,GlobalUavData):
        """
        """
        vel = [0,0,0]
        curr_vel = self.__velocity
        vel[0] = 100000*(self.__droplocation[0]-(vehicle[j].get_pos()[0])) 
        vel[1] = 100000*((self.__droplocation[1])-(vehicle[j].get_pos()[1]))
        vel[2] = 0
        vel = self.inter_uav_collsion(vel,GlobalUavData)
        vc = inside_circle(vel)

        return vc


    def update_pbest(self):
        max_temp = max(self.__personal_humans, key=lambda a: self.__personal_humans[2])
        self.__pbest = max_temp[2]
        self.__bestlocation = [max_temp[0], max_temp[1]]


    def payload_drop(self, GlobalUavData, loc):  
        """
        """
        print("dropping payload...")
        print("...")
        friend_distances = {i:sys.maxsize for i in range(1,1+len(GlobalUavData))}
        temp = []
        for friend in GlobalUavData:
            friend = str(friend)
            friend_location = GlobalUavData[self.id][friend]["GPS"]
            dist = distance(loc[0], loc[1], friend_location[0], friend_location[1])
            if self.__payload == 1 : # set flag
                if dist <= 2.5*fol:
                    friend_distances[friend] = dist
            if self.__payload ==0:
                if dist <=2.5*fol and distance(self__payload[0],self__payload[1],self.get_pos()[0],self.get_pos()[1])>distance(loc[0], loc[1], self.get_pos()[0],self.get_pos()[1]):
                    friend_distances[friend] = dist
                

        min_dist = min(friend_distances.values())
        if min_dist != sys.maxsize:            
            critical_key = min(friend_distances.items(), key=lambda x: friend_distances.items()[1])[0]
            if critical_key == self.id:
                # drop payload using the current uav
                if self.__payload == 0:
                    self.__droplocation = [loc[0], loc[1], 5]
                else:
                    newloc=self.__droplocation
                    self.__droplocation = [loc[0], loc[1], 5]

                    self.payload_drop(GlobalUavData,newloc)



    """def probablityfn(self):
        k=1
        p=0.75

        for i in self.__global_humans:
            distancefactor=0
            confidencefactor=0
            for j in self.__global_humans:
                dist=distance(j[0],j[1],i[0],i[1])
                if dist:
                    distancefactor=distancefactor+k*(1/dist)
                confidencefactor=p*i[2]+confidencefactor
            probablity=0.7+distancefactor+confidencefactor
            if len(i)>=5:
                i[4] = probablity 
            else:
                i.append(probablity)



        num = len(self.__global_humans)
        dp_array = [[0 for in range(num)] for i in range(num)]

        for i in range(num):
            dp_array[]"""

    ## Function to transmit the necessary data to other UAVs
    def transmitdata(self):
        pass


    ## Function to receive the necessary data from other UAVs
    def recievedata(self):
        pass





    # add swarm code to class--> aniket swarm

    # add payload_drop to class


