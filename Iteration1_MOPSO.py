import os
import time
import dronekit
import math
import random
import pickle
from math import *
from random import *
import numpy as np
import threading
from dronekit import *
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

class swarm_bot:
    def __init__(self,s):
        self.string=str(s)
        self.velocity = [0,0]
        self.vehicle = connect(self.string)#, wait_ready=True)
        self.id=int(self.vehicle.parameters['SYSID_THISMAV'])
        print("Connected to vehicle id : " + str(self.id))

    def get_UAV_ID(self):
        return self.id

    def get_pos(self):
        self.pos = [self.vehicle.location.global_frame.lat,self.vehicle.location.global_frame.lon]
        return self.pos

    def get_alt(self):
        return self.vehicle.location.global_relative_frame.alt

    def get_vel(self):
        return self.vehicle.velocity

    def update_pos(self,pos):
        self.position=pos
        pos_x = pos[0]
        pos_y = pos[1]
        pos_z = 30

        a_location = LocationGlobalRelative(pos_x, pos_y, pos_z)
        self.vehicle.simple_goto(a_location, 15, 15)

    def update_vel(self,v):
        self.velocity=v
        velocity_x=v[0]
        velocity_y=v[1]
        velocity_z=0
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
            velocity_x, # X velocity in NED frame in m/s
            velocity_y, # Y velocity in NED frame in m/s
            velocity_z, # Z velocity in NED frame in m/s
            0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        self.vehicle.send_mavlink(msg)

    def arm_and_takeoff(self, aTargetAltitude):

        while not self.vehicle.is_armable:
            time.sleep(1)

        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            time.sleep(1)

        self.vehicle.simple_takeoff(aTargetAltitude)

        while True:
            if self.vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.90:
                print ("Reached target altitude",self.id)
                break
            time.sleep(1)

    def land(self):
        self.vehicle.mode = VehicleMode("LAND")

def distance(lat1, lat2, lon1, lon2):
    # The math module contains a function named
    # radians which converts from degrees to radians.
    lon1 = radians(lon1)
    lon2 = radians(lon2)
    lat1 = radians(lat1)
    lat2 = radians(lat2)
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
    c = 2 * asin(sqrt(a))
    r = 6371
    return(c * r)

def Point_from_dist_and_initial_wp(lat1, lon1, angle, d):
    # Return final coordinates (lat2,lon2) [in degrees] given initial coordinates
    # (lat1,lon1) [in degrees] and a bearing [in degrees] and distance [in km]
    lat1 = radians(lat1)
    lon1 = radians(lon1)
    R = 6371
    d = d / 1000
    ad = d / R
    lat = asin(sin(lat1) * cos(ad) + cos(lat1) * sin(ad) * cos(angle))
    lon = lon1 + atan2(sin(angle) * sin(ad) * cos(lat1), cos(ad) - sin(lat1) * sin(lat))
    lat = degrees(lat)
    lon = degrees(lon)
    return (lat, lon)

def initial_wps_func(Num_of_UAVs, lat, lon, heading, length, breadth):
    wps = list()
    #wps.append([lat,lon])
    for i in range(Num_of_UAVs):
        wps.append(Point_from_dist_and_initial_wp(lat, lon, heading + pi/2, breadth/Num_of_UAVs*i+breadth/(2*Num_of_UAVs)))

    return wps

def final_wps_func(Num_of_UAVs, lat, lon, heading, length, breadth):
    wps = list()
    #wps.append([lat,lon])
    for i in range(Num_of_UAVs):
        wps.append(Point_from_dist_and_initial_wp(lat, lon, heading + pi/2, breadth/Num_of_UAVs*i+breadth/(2*Num_of_UAVs)))

    return wps

def get_bounding_rectangle(lat, lon, heading, length, breadth):
    bounding_rectangle_wps = list()
    bounding_rectangle_wps.append([lat,lon])
    bounding_rectangle_wps.append(Point_from_dist_and_initial_wp(lat, lon, heading + pi/2, breadth))
    bounding_rectangle_wps.append(Point_from_dist_and_initial_wp(lat, lon, heading + pi/4, sqrt(breadth**2 + length**2)))
    bounding_rectangle_wps.append(Point_from_dist_and_initial_wp(lat, lon, heading, length))

    return bounding_rectangle_wps

def get_bounding_rectangle_with_initial_point_at_centre(lat, lon, heading, length, breadth):
    bounding_rectangle_wps = list()

    theta1 = 2*atan2(length/2, breadth/2)
    theta2 = 2*atan2(breadth/2, length/2)
    d = sqrt((breadth/2)**2 + (length/2)**2)
    bounding_rectangle_wps.append(Point_from_dist_and_initial_wp(lat, lon, heading + theta1 + 1.5*theta2, d))
    bounding_rectangle_wps.append(Point_from_dist_and_initial_wp(lat, lon, heading + theta1 + 0.5*theta2, d))
    bounding_rectangle_wps.append(Point_from_dist_and_initial_wp(lat, lon, heading + 0.5*theta2, d))
    bounding_rectangle_wps.append(Point_from_dist_and_initial_wp(lat, lon, heading + 2*theta1 + 1.5*theta2, d))

    return bounding_rectangle_wps

def PSO(vel, Pbest, Gbest, pos):
    w_k = 0.4
    c1 = 0.4
    c2 = 0.4
    r1 = 1000
    r2 = 1000
    #print("Pbest: ",Pbest)
    #print("Gbest: ",Gbest)
    return [w_k*vel[0] + c1*r1*(Pbest[0]-pos[0]) + c2*r2*(Gbest[0]-pos[0]),w_k*vel[1] + c1*r1*(Pbest[1]-pos[1]) + c2*r2*(Gbest[1]-pos[1])]

def Rep_vel(pos_i, pos_j):
    rep_gain = ((d-0.015)/d) * 15000
    return [rep_gain * (pos_j[0]-pos_i[0]),rep_gain * (pos_j[1]-pos_i[1])]

def Flock_vel(pos_i, final_wp):
    flock_gain = 10000
    return [flock_gain * (final_wp[0]-pos_i[0]), flock_gain * (final_wp[1]-pos_i[1])]

def Found_humans(Humans_list_with_probabilities, pos, bearing):
    L = len(Humans_list_with_probabilities)
    FOV_X = 68
    FOV_Y = 37
    Found_humans_list = list()

    bounding_rectangle_wps = get_bounding_rectangle_with_initial_point_at_centre(pos[0], pos[1], bearing, FOV_X, FOV_Y)
    for i in range(L):
        point = Point(Humans_list_with_probabilities[i][0],Humans_list_with_probabilities[i][1])
        polygon = Polygon(bounding_rectangle_wps)
        if polygon.contains(point) == True:
            Found_humans_list.append(Humans_list_with_probabilities[i])

    Pbest = [0,0,0]
    if len(Found_humans_list) !=0:
        Pbest_prob = Found_humans_list[0][2]
        for i in range(len(Found_humans_list)):
            if Found_humans_list[i][2] >= Pbest_prob:
                Pbest_prob = Found_humans_list[i][2]
                Pbest = Found_humans_list[i]

    if len(Found_humans_list)!=0:
        return Found_humans_list, Pbest, 1
    else:
        return [0,0,0], Pbest, 0

def Loadfile(list_to_be_uploaded, name):
    file = open("/home/aniket/"+name,"w")
    for element in list_to_be_uploaded:
        file.write(str(element[0])+" "+str(element[1])+" 20")
        file.write("\n")
    file.close()

def Generate_Humans_List(start_point, l, b):
    center_point = Point_from_dist_and_initial_wp(start_point[0], start_point[1], pi/4, sqrt((l/2)**2 + (b/2)**2))
    Humans_list_with_probabilities=list()
    for _ in range(25):
        theta = radians(randint(0,360))
        d = randint(0, 500)
        prob = uniform(0.5,1)
        point = Point_from_dist_and_initial_wp(center_point[0], center_point[1], theta, d)
        Humans_list_with_probabilities.append([point[0],point[1],prob])

    Loadfile(Humans_list_with_probabilities, "obs.txt")

    return Humans_list_with_probabilities

def Limit_vel(v, v_max):
    return (v / abs(v)) * min(abs(v), abs(v_max))

if __name__=="__main__":
    vehicle = list()
    vehicle_thread = list()

    goal_wp = [28.759146,77.113238]
    heading = 0
    N = 20
    l = 1000
    b = 1000
    v_max = 5
    takeoff_alti = 30
    #Humans_list_with_probabilities = Generate_Humans_List(goal_wp, l, b)
    #Humans_list_with_probabilities_file = open("Random_humans_list", "wb")
    #pickle.dump(Humans_list_with_probabilities, Humans_list_with_probabilities_file)
    #Humans_list_with_probabilities_file.close()

    Humans_list_with_probabilities_file = open('Random_humans_list', 'rb')
    Humans_list_with_probabilities = pickle.load(Humans_list_with_probabilities_file)


    initial_wps = initial_wps_func(N, goal_wp[0], goal_wp[1], heading, l, b)
    bounding_rectangle_wps = get_bounding_rectangle(goal_wp[0], goal_wp[1], heading, l, b)
    final_wps = final_wps_func(N, bounding_rectangle_wps[3][0], bounding_rectangle_wps[3][1], heading, l, b)
    Loadfile(bounding_rectangle_wps, "cells.txt")

    print(final_wps)
    time.sleep(50)
    for i in range(N):
        vehicle.append(swarm_bot("127.0.0.1:" + str(14550 + i * 10)))

    for i in range(N):
        vehicle_thread.append(threading.Thread(target=vehicle[i].arm_and_takeoff, args=(takeoff_alti,)))
        vehicle_thread[i].start()

    # =====================================================
    # Method to wait for all vehicles to reach takeoff_alti
    # =====================================================
    discard_list = list()
    while True:
        for i in range(N):
            if i in discard_list:
                continue
            elif vehicle[i].get_alt() >= 0.9*5:
                discard_list.append(i)

        if len(discard_list)==N:
            break

    discard_list.clear()

    for i in range(N):
        vehicle[i].update_pos(initial_wps[i])

    # =====================================================
    # Method to wait for all vehicles to reach initial_wps
    # =====================================================
    while True:
        for i in range(N):
            if i in discard_list:
                continue

            pos = vehicle[i].get_pos()
            d = distance(pos[0], initial_wps[i][0], pos[1], initial_wps[i][1])
            if d<0.005:
                discard_list.append(i)

        if len(discard_list)==N:
            break

    d_matrix = list()
    flag_list = [0]*N
    temp_flag_list = [0]*N
    Pbest = np.zeros((N,3))

    for i in range(N):
        prit
        Pbest[i][0], Pbest[i][1] = final_wps[i][0], final_wps[i][1]

    print("Printing Pbest 1st time", Pbest)
    while True:
        for i in range(N):
            pos_i = vehicle[i].get_pos()
            d_L = distance(pos_i[0], final_wps[i][0], pos_i[1], final_wps[i][1])
            v = Flock_vel(pos_i, final_wps[i])
            v[0] = Limit_vel(v[0], v_max)
            v[1] = Limit_vel(v[1], v_max)
            print("V_flocking for UAV", i + 1, " is:", v)

            for j in range(N):
                if i==j:
                    continue

                pos_j = vehicle[j].get_pos()
                d = distance(pos_i[0], pos_j[0], pos_i[1], pos_j[1])
                d_matrix.append([d,j])

                if d < 0.015:
                    v_rep = Rep_vel(pos_i, pos_j)
                    print("V_repulsion between UAV",i+1," and",j+1," is:", v_rep)
                    v = [v[0]+v_rep[0], v[1]+v_rep[1]]

            d_matrix.sort()
            Found_humans_list, Pbest_temp, temp_if_flag = Found_humans(Humans_list_with_probabilities, pos_i, radians(vehicle[i].vehicle.heading))
            if Pbest_temp != [0,0,0]:
                Pbest[i] = Pbest_temp

            #print("Pbest:", Pbest)
            if  temp_if_flag == True:
                #print("UAV",i+1," found",len(Found_humans_list)," Humans")
                flag_list[i],flag_list[d_matrix[0][1]],flag_list[d_matrix[1][1]],flag_list[d_matrix[2][1]],flag_list[d_matrix[3][1]] = [1]*5

            if flag_list[i] == 1:
                Gbest_prob = Pbest[0][2]
                Gbest = Pbest[0]
                for i in range(N):
                    if Pbest[i][2] > Gbest_prob:
                        Gbest_prob = Pbest[i][2]
                        Gbest = Pbest[i]

                #print("Gbest:",Gbest)
                if Pbest[i][2] == 0:
                    Pbest[i][0] = final_wps[i][0]
                    Pbest[i][0] = final_wps[i][1]

                print("Pbest", Pbest)
                print("Gbest", Gbest)
                v_PSO = PSO(v, Pbest[i], Gbest, pos_i)
                print("V_PSO for UAV", i + 1, " is:", v_PSO)
                v = [v[0]+v_PSO[0], v[1]+v_PSO[1]]
                print("Final velocity for UAV",i+1," is",v)
                #print("PSO Velocity of vehicle", i + 1, " = ", v_PSO)

            #v[0] = Limit_vel(v[0], v_max)
            #v[1] = Limit_vel(v[1], v_max)
            vehicle[i].update_vel(v)
            #print("Velocity of vehicle", i+1, " = ",v )
        print("\n\n\n")
        time.sleep(1)