
""""from math import radians, cos, sin, asin, sqrt
import dronekit
from pymavlink import mavutil
import numpy as np
from math import *
n=22#int(input("enter no of uavs"))
a=[]
for i in range(2):
	k=50#int(input("enter the coordinates"))
	a.append(k)
print(a)
lb=[]
for i in range(2):
	k=40#int(input("enter the length and breadth of the search area"))
	lb.append(k)
coordinates=[[a[0]-lb[0]/2,a[1]-lb[1]/2],[a[0]-lb[0]/2,a[1]+lb[1]/2],[a[0]+lb[0]/2,a[1]-lb[1]/2],[a[0]+lb[0]/2,a[1]+lb[1]/2]]
print(coordinates)
intialwaypoints=[]
finalwaypoints=[]
k=(coordinates[2][0]-coordinates[0][0])/n
for i in range(n):
	j=k*i
	u=coordinates[0]
	u2=coordinates[3][1]
	p=[u[0]+k/2+j,u[1]]
	q=[u[0]+k/2+j,u2]
	intialwaypoints.append(p)
	finalwaypoints.append(q)

print(intialwaypoints)
uavs=[]
for i in range(1,22):
	uavi={'intial position': intialwaypoints[i-1]}
	uavs.append(uavi)
listt=[]
print(coordinates)
a1=int(a[0]-lb[0]/2)
a2=int(a[0]+lb[0]/2)
a3=int(a[1]-lb[1]/2)
a4=int(a[1]+lb[1]/2)
for i in range(a1,a2,1):
	for j in range(a3,a4,1):
		listt.append([i,j])
for i in uavs:
	i['final position']=finalwaypoints[i-1]
len(intialwaypoints)

"""
import dronekit
from pymavlink import mavutil
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
import numpy as np
from math import *
import threading
from math import radians, cos, sin, asin, sqrt,atan2
import random
class swarmbot:
    def __init__(self,s):
        self.velocity=[0,0]
        self.vehicle = dronekit.connect(s)
        self.id=int(self.vehicle.parameters['SYSID_THISMAV'])
        print("Connected to vehicle id : " + str(self.id))
        self.wplist=[]
    
    def get_pos(self):
        self.pos= [self.vehicle.location.global_frame.lat,self.vehicle.location.global_frame.lon]
        return self.pos
    
    def update_pos(self,pos,v):
        self.position=pos
        pos_x = pos[0]
        pos_y = pos[1]
        pos_z = 25

        a_location = LocationGlobalRelative(pos_x, pos_y, pos_z)
        self.vehicle.simple_goto(a_location,v,v)

    def update_vel(self,v):
        self.velocity=v       
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

    def heading(self):
        self.head=self.vehicle.heading
        return self.head

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

    def wpoints(self,p):
        self.wplist=[]
        for i in range(len(p)):
           self.wplist.append(p[i])
        return self.wplist

    def waypoints(self):
        return self.wplist

def distance(lat1,lon1,lat2,lon2): 
    lon1=radians(lon1) 
    lon2=radians(lon2) 
    lat1=radians(lat1) 
    lat2=radians(lat2) 
    dlon=lon2-lon1  
    dlat=lat2-lat1 
    a=sin(dlat/2)**2+cos(lat1)*cos(lat2)*sin(dlon/2)**2
    c=2*asin(sqrt(a))
    R=6371 
    return(c*R)
def pointRadialDistance(lat1,lon1,angle,d):
    """
    Return final coordinates (lat2,lon2) [in degrees] given initial coordinates
    (lat1,lon1) [in degrees] and a bearing [in degrees] and distance [in km]
    """
    lat1 = radians(lat1)
    lon1 = radians(lon1)
    R=6371
    d=d/1000
    ad=d/R
    lat2 = asin(sin(lat1)*cos(ad) +cos(lat1)*sin(ad)*cos(angle))
    lon2 = lon1 + atan2(sin(angle)*sin(ad)*cos(lat1),cos(ad)-sin(lat1)*sin(lat2))
    lat = degrees(lat2)
    lon = degrees(lon2)
    b=[lat,lon]
    return b

def rectangle(m,n,lat,lon,heading):
    heading=float(heading)
    heading=radians(heading)
    e=pointRadialDistance(lat,lon,heading,n)
    rheading=heading+pi/2
    lheading=heading-pi/2
    p1=pointRadialDistance(lat,lon,lheading,m/2)
    p2=pointRadialDistance(lat,lon,rheading,m/2)
    p3=pointRadialDistance(e[0],e[1],rheading,m/2)
    p4=pointRadialDistance(e[0],e[1],lheading,m/2)
    return p1,p2,p3,p4

def bearing(lat1,lon1,lat2,lon2):
    lon1=radians(lon1)
    lon2=radians(lon2)
    lat1=radians(lat1)
    lat2=radians(lat2)
    dlon=abs(lon2-lon1)
    bearing=atan2(sin(dlon)*cos(lat2),cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(dlon))
    return (degrees(bearing))
def intialwaypoints(n,p1,p2,p3,p4,x):
	dis=float(distance(p1[0],p1[1],p2[0],p2[1])/n)
	a=[]
	for i in range(n):
	   if i==0:
            uavi={}
            uavi["intialwaypoints"]=pointRadialDistance(p1[0],p1[1],x,(dis/2))
            uavi["finaalwaypoints"]=pointRadialDistance(p3[0],p3[1],x,(dis/2))
            a.append(uavi)
            y=pointRadialDistance(p1[0],p1[1],x,(dis/2))
            z=pointRadialDistance(p3[0],p3[1],x,(dis/2))
	   else:
            uavi={}
            uavi["intialwaypoints"]=pointRadialDistance(y[0],y[1],x,dis)
            uavi["finaalwaypoints"]=pointRadialDistance(z[0],z[1],x,dis)
            a.append(uavi)
            y=pointRadialDistance(y[0],y[1],x,dis)
            z=pointRadialDistance(z[0],z[1],x,dis)
	return a
def minimumdistance(j):
    m=[]
    real=[]
    index=[]
    x=vehicle[j].get_pos()
    for i in range(n):
        y=vehicle[i].get_pos()
        m.append(distance(x[0],x[1],y[0],y[1]))
    a=m.sort(reverse=True)
    for i in range(4):
        q=a[i]
        real.append(q)
    for i in real:
        j=m.index(i)
        index.append(j)
    return index
def updategbest(x):
    a=[]
    b=[]
    for i in x:
        a.append(vehiclei["pbest"])
        b.append(vehiclei["gbest"])
    m=a.sort(reverse=True)
    n=b.sort(reverse=True)
    if m[0]>n[0]:
        gbest=m[0]
        y=a.index(gbest)
        q=x[y]
        u=vehicleq["bestlocation"]
    else:
        gbest=n[0]
        y=b.index(gbest)
        q=x[y]
        u=vehicleq["gbestloc"]
    for i in x:
        vehiclei["gbest"]=gbest
        vehiclei["gbestloc"]=u

def generatevelocity(v,i,time):
    vel=[]
    T=300
    R1=((random.randrange(0,100,10))/100)
    R2=((random.randrange(0,100,10))/100)
    w=wstart-(wstart-wend)*(pow(time/T,2))
    vel[0] = w*v[0] + R1*1000*(vehiclei["bestlocation"][0]-vehicle[i].get_pos()[0]) + R2*1000*(vehiclei["gbestloc"][0]-vehicle[i].get_pos()[0])
    vel[1] = w*v[1] + R1*1000*(vehiclei["bestlocation"][1]-vehicle[i].get_pos()[1]) + R2*1000*(vehiclei["gbestloc"][1]-vehicle[i].get_pos()[1])
    vel[2]=0
    return vel






"""def spacearea(p1,p4):
    listt=[]
    i=p1[0]
    while i==p4[0]:
        if i==p1[0]:
            j=p1[1]
            while j==p4[1]:
                k=[i,j]
                listt.append(k)
                u=pointRadialDistance(i,j,heading,0.1)
                j=u[1]
        else:
            j=p[1]
            while j==p4[1]:
                k=[i,j]
                listt.append(k)
                u=pointRadialDistance(i,j,heading,0.1)
                j=u[1]
    p=pointRadialDistance(i,j,x,0.1)
    i=p[0]
return listt
def pointsremoval(lat,lon,x,u):
    a1,a2,a3,a4=rectangle(fol,fow,lat,lon,x)
    for i in u:
        if a1[0]>a4[0]:
            if a1[1]>a4[1]:
                if i[0]>=a1[0] and i[0]<=a4[0] and i[1]>=a1[1] and i[1]<=a4[1]:
                    u.remove(i)
            if a1[1]<a4[1]:
                if i[0]>=a1[0] and i[0]<=a4[0] and i[1]<=a1[1] and i[1]>=a4[1]
"""






sal=1000#int(input("enter the length of the seach area "))
saw=1000#int(input("enter the width of search area"))
fol=100#int(inpu("enter the length of fov"))
fow=50#int(input("enter the width of the fov"))
lat=28.763638#float(input("enter the latitude"))
lon=77.114030#float(input("enter the longitude"))
heading=input("Enter the heading angle of uavs(in degrees):")
p1,p2,p3,p4=rectangle(sal,saw,lat,lon,heading)
n=10#int(input("enter the no of uavs "))
uavs=[]
x=bearing(p1[0],p1[1],p2[0],p2[1])
uavs=intialwaypoints(n,p1,p2,p3,p4,x)
print(p1,p4)
for i in range(n):
	z=uavs[i]
	y=z["intialwaypoints"]
	print(y)
	print(i)






