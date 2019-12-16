import dronekit
from pymavlink import mavutil
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
import numpy as np
from math import *
import threading
from math import radians, cos, sin, asin, sqrt,atan2
import random
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from shapely.geometry import LineString
from helper import *
from time import time
from SwarmBot import SwarmBot
import sys

alti = 5
TMAX = 500
sal=1000#int(input("enter the length of the seach area "))
saw=1000#int(input("enter the width of search area"))
fol=100#int(inpu("enter the length of fov"))
fow=100#int(input("enter the width of the fov"))
lat=28.763638#float(input("enter the latitude"))
lon=77.114030#float(input("enter the longitude"))
heading=0#input("Enter the heading angle of uavs(in degrees):")
p1,p2,p3,p4=rectangle(sal,saw,lat,lon,heading)
n=10#int(input("enter the no of uavs "))
x=bearing(p1[0],p1[1],p2[0],p2[1])
INITIALWAY=intialwaypoints(n,p1,p2,p3,p4,x)
finalwaypoints=finalwaypoins(n,p1,p2,p3,p4,x)

Id = sys.argv[1]
self_connection_string = '127.0.0.1:' + str(14551 + 10*Id)
local_address="127.0.0.1"
local_port=11111 + Id
sock = create_and_bind_uav_ports((local_address,local_port)):

#GlobalUavData = {{i:{"GPS":(), "Humans":[], "G":0, "P":0, "bestloc":[], "gbestloc":[]}} for i in range(1,n+1)}
detected_humans = []

uav = SwarmBot(self_connection_string)
uav.arm_and_takeoff(5)
uav.update_pos([finalwaypoints[uav.id-1][0], finalwaypoints[uav.id-1][1],5],5)
wstart=0.9
wend=0.4

timer = 0
while True:
    GlobalUavData = recv_data(soc)

    if timer == TMAX or distance(finalwaypoints[uav.id-1][0], finalwaypoints[uav.id-1][1], uav.get_pos()[0], uav.get_pos()[1]):
        #fix this
        print("ending mopso search...")
        print("...")
        break

    # for testing
    a1,a2,a3,a4=rectangle2(uav.get_pos(),uav.heading(),fol,fow)
    for i in detected_humans:
        point = Point(i[0], i[1])
        polygon = Polygon([(a1[0],a1[1]), (a2[0],a2[1]), (a3[0],a3[1]), (a4[0],a4[1])])
        if polygon.contains(point):
            uav.update_PersonalHumans(i)
            GlobalUavData[uav.id]["Humans"].append(i)

    # shared human list update
    shared_human_list = set()
    for i in GlobalUavData:
        for humans in GlobalUavData[i]["Humans"]:
            shared_human_list.add(humans)    # multiple detection

        shared_human_list = list(shared_human_list)

    # gbest and p best updation
    for humans in shared_human_list:
        nearestuav = uav.minimumdist(GlobalUavData)
        uav.update_pbest() 
        uav.updategbest(GlobalUavData, nearestuav)

        uav.payload_drop(GlobalUavData, humans)

    
    # check uav state
    if uav.get_state() == 0:
        uav.checkifpayload()
        vel = uav.velocity_post_drop()
        uav.update_vel(vel)
        print(vel)

    else:
        vel = uav.generate_mopso_velocity(timer, finalwaypoints, num, TMAX, wstart, wend)
        uav.update_vel(vel)
        print(vel)
        uav.change_gpbest(finalwaypoints, p1, p4)

    timer += 1
    time.sleep(1)
