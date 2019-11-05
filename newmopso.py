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
    
    def get_vel():
        return self.vehicle.velocity

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

def rectangle(m,n,lat,lon,head):
    heading=float(head)
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
    dis=1000*float(distance(p1[0],p1[1],p2[0],p2[1])/n)
    a=[]
    for i in range(n):
        if i==0:
            u=pointRadialDistance(p1[0],p1[1],x,(dis/2))
            print(u)
            a.append(u)
        else:
            nmmm=pointRadialDistance(a[i-1][0],a[i-1][1],x,(dis))
            print(nmmm)
            a.append(nmmm)
    return a
def finalwaypoins(n,p1,p2,p3,p4,x):
    b=[]
    dis=1000*float(distance(p1[0],p1[1],p2[0],p2[1])/n)
    for i in range(n):
        if i==0:
            u=pointRadialDistance(p3[0],p3[1],x,(dis/2))
            print(u)
            b.append(u)
        else:
            nmmm=pointRadialDistance(b[i-1][0],b[i-1][1],x,(dis))
            print(nmmm)
            b.append(nmmm)
    return b



def minimumdistance(j):
    m=[]
    real=[]
    index=[]
    x=vehicle[j].get_pos()
    for i in range(n):
        y=vehicle[i].get_pos()
        m.append(distance(x[0],x[1],y[0],y[1]))
    a=[]
    for i in m:
        a.append(i)

    a.sort()
    for i in range(5):
        q=a[i]
        real.append(q)
    for i in real:
        j=m.index(i)
        index.append(j)
    return index
def updategbest(x):
    a=[]
    b=[]
    for j in x:
        a.append(uavs[j]["pbest"])
        b.append(uavs[j]["gbest"])
        #print(a,b)
    m=[]
    n=[]
    for i in a:
        m.append(i)
    for i in b:
        n.append(i)

    m.sort(reverse=True)
    n.sort(reverse=True)
    if m[0]>n[0]:
        gbest=m[0]
        y=a.index(m[0])
        q=x[y]
        u=uavs[q]["bestlocation"]
        #print(uavs[q]["bestloaction"])
    else:
        gbest=n[0]
        y=b.index(n[0])
        q=x[y]

        u=uavs[q]["gbestloc"]
        #print(uavs[q]["gbestloc"])
    
    for i in x:
        uavs[i]["gbest"]=gbest
        uavs[i]["gbestloc"]=u
        #print(uavs[i])

def generatevelocity(v,i,timer):
    vel=[0,0,0]
    T=300
    R1=(random.randrange(0,100,10)/100)

    R2=(random.randrange(0,100,10)/100)
    w=(wstart-(wstart-wend)*(pow(timer/T,2)))
    print(R1,R2,w)
    n=uavs[i]["bestlocation"]
    m=uavs[i]["gbestloc"]
    if n==[0,0]:

        vel[0] = w*(v[0]) +  R2*1000*(m[0]-vehicle[i].get_pos()[0])
        vel[1] = w*(v[1]) +  R2*1000*(m[1]-vehicle[i].get_pos()[1])
        vel[2]=0
    else:
        vel[0] = w*(v[0]) + R1*1000*(n[0]-vehicle[i].get_pos()[0]) + R2*1000*(m[0]-vehicle[i].get_pos()[0])
        vel[2]=0
        vel[1] = w*(v[1]) + R1*1000*(n[1]-vehicle[i].get_pos()[1]) + R2*1000*(m[1]-vehicle[i].get_pos()[1])
    print(vel)
    return vel






def spacearea(p1,p2,p3,sal,saw,heading):
    listt=[]
    i=p1[0]
    j=p1[1]
    x=sal/2
    y=saw/2
    heading=float(heading)
    heading=radians(heading)
    head=heading+pi/2
    print(x,y)
    counter=0
    p=True
    q=True
    while p==True:
        a=pointRadialDistance(i,j,head,2)
        count=0
        if counter==y:
            break
        while q==True:
            if count==x:
                break
            listt.append([i,j])
            z=pointRadialDistance(i,j,heading,2)
            j=z[1]
            i=z[0]
            count=count+1
        i=a[0]
        j=a[1]
        counter=counter+1
    return listt
"""
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
INITIALWAY=intialwaypoints(n,p1,p2,p3,p4,x)
finalwaypoins=finalwaypoins(n,p1,p2,p3,p4,x)
print(p1,p4)
humans=[]
humans=spacearea(p1,p2,p3,sal,saw,heading)
print(len(humans))
indexs=[]
for i in range(30):
    indexw=random.randrange(0,250000-1)
    indexs.append(indexw)
loooopy=[]

for i in indexs:
    cods=humans[i]
    point=[cods[0],cods[1],(random.randrange(70,100,1)/100)]
    loooopy.append(point)
    humans.remove(cods)
vehicle=list()
for i in range(n):
    vehicle.append(swarmbot("127.0.0.1:" + str(14550 + i*10)))

for i in range(n):
    vehicle[i].arm_and_takeoff(5)
for i in range (n):
    loi=[finalwaypoins[i][0],finalwaypoins[i][1],5]
    vehicle[i].update_pos(loi,5)
uavs=[]

for j in range(n):

    veh={}
    veh["vehicleno"]=j
    veh["pbest"]=0
    veh["gbest"]=0
    veh["bestlocation"]=[0,0]
    veh["gbestloc"]=0
    veh["velocity"]=5
    uavs.append(veh)
extralist=[]

wstart=0.9
wend=0.4
T=300#(int(input("enter mopso time")))
timer=0.1
while True:
    for j in range(n):
        pos1=vehicle[j].get_pos()
        header=vehicle[j].vehicle.heading
        a1,a2,a3,a4=rectangle(fol,fow,pos1[0],pos1[1],header)
        for i in loooopy:
            if a1[0]<a4[0]:
                if a1[1]>a4[1]:
                    if i[0]>=a1[0] and i[0]<=a4[0] and i[1]<=a1[1] and i[1]>=a4[1]:
                        if i[2]>uavs[j]["pbest"]:
                            uavs[j]["pbest"]=i[2]
                            uavs[j]["bestlocation"]=[i[0],i[1]]
                            print("block1")
                            print(i)
                            print(uavs[j])
                            x=minimumdistance(j)
                            print(x)
                            updategbest(x)
                            for k in x:
                                v=[]
                                v=vehicle[k].get_vel()
                                print(v)
                                print(uavs[k])
                                vel=generatevelocity(v,k,time)
                                vehicle[k].update_vel(vel)


                if a1[1]<=a4[1]:
                    if i[0]>=a1[0] and i[0]<=a4[0] and i[1]>=a1[1] and i[1]<=a4[1]:
                        if i[2]>uavs[j]["pbest"]:
                            uavs[j]["pbest"]=i[2]
                            uavs[j]["bestlocation"]=[i[0],i[1]]
                            print(i)
                            print(uavs[j])
                            print("block2")
                            x=minimumdistance(j)
                            print(x)
                            updategbest(x)
                            for k in x:
                                v=[]
                                v=vehicle[k].vehicle.velocity
                                print(v)
                                print(uavs[k])
                                vel=generatevelocity(v,k,timer)
                                uavs[k]["velocity"]=[vel[0],vel[1],vel[2]]
                                extralist.append(k)
                                vehicle[k].update_vel(vel)
                        
                        


            if a1[0]>a4[0]:
                if a1[1]>=a4[1]:
                    if i[0]<=a1[0] and i[0]>=a4[0] and i[1]<=a1[1] and i[1]>=a4[1]:
                        if i[2]>uavs[j]["pbest"]:
                            uavs[j]["pbest"]=i[2]
                            uavs[j]["bestlocation"]=[i[0],i[1]]
                            print(i)
                            print(uavs[j])
                            print("block3")
                            x=minimumdistance(j)
                            print(x)
                            updategbest(x)
                            for k in x:
                                v=[]
                                v=vehicle[k].vehicle.velocity
                                print(v)
                                print(uavs[k])
                                vel=generatevelocity(v,k,timer)
                                vehicle[k].update_vel(vel)

                      
                if a1[1]<=a4[1]:
                    if i[0]<=a1[0] and i[0]>=a4[0] and i[1]>=a1[1] and i[1]<=a4[1]:
                        if i[2]>uavs[j]["pbest"]:
                            uavs[j]["pbest"]=i[2]
                            uavs[j]["bestlocation"]=[i[0],i[1]]
                            print(uavs[j])
                            print(i)
                            print("block4")
                            x=minimumdistance(j)
                            print(x)
                            updategbest(x)
                            for k in x:
                                v=[]
                                v=vehicle[k].vehicle.velocity
                                print(v)
                                print(uavs[k])
                                vel=generatevelocity(v,k,timer)
                                vehicle[k].update_vel(vel)
    extralist=list(set(extralist))
    for jum in extralist:
        vehicle[jum].update_vel(uavs[jum]["velocity"])

                                
                       
        
    if timer==300:
        print("end mopso")
        break
    timer=timer+0.1
    time.sleep(0.1)


