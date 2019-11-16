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
    
    def get_vel(self):
        return self.vehicle.velocity

    def update_pos(self,pos,v):
        self.position=pos
        pos_x = pos[0]
        pos_y = pos[1]
        pos_z = 5

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
def rectangle2(pos1,heading,fol,fow):
    u=pointRadialDistance(pos1[0],pos1[1],heading-pi,fow/2)
    n=[]
    n=rectangle(fol,fow,u[0],u[1],heading)
    return n
def bearing(lat1,lon1,lat2,lon2):
    lon1=radians(lon1)
    lon2=radians(lon2)
    lat1=radians(lat1)
    lat2=radians(lat2)
    dlon=abs(lon2-lon1)
    bearing=atan2(sin(dlon)*cos(lat2),cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(dlon))
    return (degrees(bearing))

def intialwaypoints(n,p1,p2,p3,p4,x):
    x=float(x)
    x=radians(x)
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
    x=float(x)
    x=radians(x)
    b=[]
    dis=1000*float(distance(p1[0],p1[1],p2[0],p2[1])/n)
    for i in range(n):
        if i==0:
            u=pointRadialDistance(p4[0],p4[1],x,(dis/2))
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
    else:
        gbest=n[0]
        y=b.index(n[0])
        q=x[y]

        u=uavs[q]["gbestloc"]
    for i in x:
        uavs[i]["gbest"]=gbest
        uavs[i]["gbestloc"]=u
        
def change_gpbest(i):
    heading123=bearing(vehicle[i].get_pos()[0],vehicle[i].get_pos()[1],uavs[i]["gbestloc"][0],uavs[i]["gbestloc"][1])
    heading234=bearing(vehicle[i].get_pos()[0],vehicle[i].get_pos()[1],uavs[i]["bestlocation"][0],uavs[i]["bestlocation"][1])
    heading=radians(float(bearing(p1[0],p1[1],p4[0],p4[1])))
    heading123=radians(float(heading123))
    heading234=radians(float(heading234))
    #print(degrees(heading),degrees(heading123),degrees(heading-pi/2),degrees(heading234),degrees(heading-pi/2))
 
    if degrees(heading123)>degrees(heading+pi/2+pi/12) or degrees(heading123)<degrees(heading-pi/2-pi/12):
        #print(degrees(heading),degrees(heading123),degrees(heading234),degrees(heading-pi/2),degrees(heading+pi/2))
        nearest=minimumdistance(i)
        #print(uavs[i]["gbest"])

        uavs[i]["gbest"]=0
        uavs[i]["gbestloc"]=[finalwaypoints[i][0],finalwaypoints[i][1]]
        #print(nearest)
        #print(uavs[i]["gbest"])
        #print("done")

    if degrees(heading234)>degrees(heading+pi/2+pi/12) or degrees(heading234)<degrees(heading-pi/2-pi/12):
        uavs[i]["pbest"]=0
        uavs[i]["bestlocation"]=[finalwaypoints[i][0],finalwaypoints[i][1]]
        #print("doneboth")








def inside_circle(vel):
    x=vel[0]
    y=vel[1]
    vc=[]
    if (x*x)+(y*y)-(25)<=0 and (x*x)+(y*y)-(4)>=0 :
        vc.append(x)
        vc.append(y)
        vc.append(0)
    elif (x*x)+(y*y)-(25)>=0:
        p = Point(0,0)
        c = p.buffer(5).boundary
        l = LineString([(0,0), (x,y)])
        i = c.intersection(l)
        vc.append(i.x)
        vc.append(i.y)
        vc.append(0)
    elif (x*x)+(y*y)-(4)<=0:
        vc.append(2*cos(atan(y/x)))
        vc.append(2*sin(atan(y/x)))
        vc.append(0)


    return vc
def payload_drop(i):
    m=[]
    a=[]
    print("pay")
    
    for j in range(n):
        if uavs[j]["flag"]==1 and uavs[j]["payloaddroping"]==0:
            y=vehicle[j].get_pos()
            a.append(j)
            m.append((1000*distance(i[0],i[1],y[0],y[1])))
    lkl=[]
    lk=[]
    for k in m:
        if float(k)>250:
            ind=m.index(k)
            lkl.append(a[ind])
            lk.append(k)

    print(lkl)
    for j in lkl:
        a.remove(j)
    for j in lk:
        m.remove(j)
    if len(m)>=1:
        u=m.index(min(m))
        print(u)
        k=a[u]
        loi=[i[0],i[1],5]
        vehicle[k].update_pos(loi,15)
        uavs[k]["payloaddroping"]=1
        uavs[k]["droplocation"]=[i[0],i[1],5]
    else:
        futurepayload.append(i)

def checkifpayload(i):

    k=vehicle[i].get_pos()
    x=k[0]
    y=k[1]
    m=uavs[i]["droplocation"]
    if (1000*distance(k[0],k[1],m[0],m[1]))<=2:
        print("payloaddroped")
        uavs[i]["payloaddroping"]=0
        uavs[i]["droplocation"]=0
        uavs[i]["flag"]=0



    
    






def generatevelocity(v,i,timer,finalwaypoints):
    vel=[0,0,0]
    T=300
    R1=(random.randrange(0,100,10)/100)
    R2=(random.randrange(0,100,10)/100)
    R3=(random.randrange(0,100,10)/100)
    w=(wstart-(wstart-wend)*(pow(timer/T,2)))
    n=uavs[i]["bestlocation"]
    m=uavs[i]["gbestloc"]
    #print(R3*1000*(finalwaypoints[i][0]-vehicle[i].get_pos()[0]),R2*1000*(m[0]-vehicle[i].get_pos()[0]),R1*1000*(n[1]-vehicle[i].get_pos()[0]) )
    if n[0]==0 and m[0]!=0:
        #print(R3*1000*(finalwaypoints[i][0]-vehicle[i].get_pos()[0]),R2*1000*(m[0]-vehicle[i].get_pos()[0]) )
        vel[0] = w*(v[0]) + R3*1000*(finalwaypoints[i][0]-vehicle[i].get_pos()[0]) + R2*1000*(m[0]-vehicle[i].get_pos()[0]) 
        vel[2]=0
        vel[1] = w*(v[1]) + R3*1000*(finalwaypoints[i][1]-vehicle[i].get_pos()[1]) + R2*1000*(m[1]-vehicle[i].get_pos()[1]) 
    elif m[0]==0 and n!=0:
        #print(R3*1000*(finalwaypoints[i][0]-vehicle[i].get_pos()[0]),R2*1000*(m[0]-vehicle[i].get_pos()[0]),R1*1000*(n[1]-vehicle[i].get_pos()[1]) )
        vel[0] = w*(v[0]) + R3*1000*(finalwaypoints[i][0]-vehicle[i].get_pos()[0]) + R1*1000*(n[0]-vehicle[i].get_pos()[0])
        vel[2]=0
        vel[1] = w*(v[1]) + R3*1000*(finalwaypoints[i][1]-vehicle[i].get_pos()[1]) + R1*1000*(n[1]-vehicle[i].get_pos()[1])

    elif n[0]==0 and m[0]==0:
        vel[0] = w*(v[0]) + R3*1000*(finalwaypoints[i][0]-vehicle[i].get_pos()[0])  
        vel[2]=0
        vel[1] = w*(v[1]) + R3*1000*(finalwaypoints[i][1]-vehicle[i].get_pos()[1]) 
    else:
        vel[0] = w*(v[0]) + R3*1000*(finalwaypoints[i][0]-vehicle[i].get_pos()[0]) + R2*1000*(m[0]-vehicle[i].get_pos()[0]) + R1*1000*(n[0]-vehicle[i].get_pos()[0])
        vel[2]=0
        vel[1] = w*(v[1]) + R3*1000*(finalwaypoints[i][1]-vehicle[i].get_pos()[1]) + R2*1000*(m[1]-vehicle[i].get_pos()[1]) + R1*1000*(n[1]-vehicle[i].get_pos()[1])

    vc=inside_circle(vel)

    

    return vc


sal=1000#int(input("enter the length of the seach area "))
saw=1000#int(input("enter the width of search area"))
fol=100#int(inpu("enter the length of fov"))
fow=100#int(input("enter the width of the fov"))
lat=28.763638#float(input("enter the latitude"))
lon=77.114030#float(input("enter the longitude"))
heading=input("Enter the heading angle of uavs(in degrees):")
p1,p2,p3,p4=rectangle(sal,saw,lat,lon,heading)
n=10#int(input("enter the no of uavs "))
x=bearing(p1[0],p1[1],p2[0],p2[1])
INITIALWAY=intialwaypoints(n,p1,p2,p3,p4,x)
finalwaypoints=finalwaypoins(n,p1,p2,p3,p4,x)
print(p1,p4)
futurepayload=[]
#
loooopy=[[28.765432768771046, 77.11045033492171, 0.78], [28.764785230054297, 77.11680462834472, 0.96], [28.765324850164422, 77.11063500000014, 0.88], [28.769136768729258, 77.11698534326479, 0.88], [28.76509102652369, 77.11094277513085, 0.79], [28.76741420291539, 77.11178266960289, 0.86], [28.767486148636138, 77.11188526382665, 0.86], [28.764749257177684, 77.11696877502442, 0.97], [28.768543216498717, 77.11659548092969, 0.72], [28.76516294516125, 77.1163737433105, 0.81], [28.765126972301672, 77.1163121883056, 0.88], [28.767540107980413, 77.1112491796393, 0.8], [28.77138705369645, 77.11335432371668, 0.92], [28.765144985816956, 77.11098381181496, 0.73], [28.771746782351197, 77.1131901670204, 0.72], [28.769046836624316, 77.11624665673511, 0.9], [28.765306863709107, 77.11094277513085, 0.8], [28.765073040091572, 77.11094277513085, 0.74], [28.76525290443132, 77.11069655502628, 0.71], [28.771189202926138, 77.11358003917407, 0.91]]

#loooopy=[[28.76405159075026, 77.11329134780544, 0.72], [28.768134510831977, 77.11341445650442, 0.95], [28.768602158091763, 77.11308616664049, 0.77], [28.771138244596663, 77.11870813056039, 0.71], [28.766533718653275, 77.10970067741864, 0.88], [28.770940394084693, 77.11550730438701, 0.92], [28.77067059760129, 77.11552782250351, 0.73], [28.765562451062106, 77.11310668475697, 0.75], [28.771497973809716, 77.11113694557336, 0.8], [28.765742314984234, 77.11840035881295, 0.92], [28.766371840824515, 77.10890047087528, 0.82], [28.766461772680415, 77.112942539825, 0.91], [28.770041072291605, 77.11798999648303, 0.8], [28.77246924104671, 77.11242958691261, 0.98], [28.769969126624886, 77.11716927182319, 0.76], [28.76487896623174, 77.11854398562842, 0.87], [28.766695596155646, 77.11483020654263, 0.94], [28.766893447141076, 77.11175248906824, 0.93], [28.768997859793167, 77.11050088396198, 0.9], [28.77225340406693, 77.10970067741864, 0.85], [28.765094803565653, 77.1165742464448, 0.88], [28.771875688324258, 77.11856450374492, 0.95], [28.766263922086463, 77.11082917382592, 0.78], [28.76550849146879, 77.11704616312421, 0.88], [28.764627156856452, 77.10959808683616, 0.74], [28.768386321207917, 77.10908513392376, 0.96], [28.765562450658468, 77.11846191316243, 0.73], [28.770346841960727, 77.11370171013536, 0.84], [28.767540958700415, 77.11171145283525, 0.71], [28.77086844849695, 77.11364015578587, 0.97]]
#loooopy=[[28.763781793847798, 77.11887227549236, 0.97], [28.765148762948655, 77.11542523192102, 0.97], [28.766569691533, 77.10949549625367, 0.83], [28.76451923802555, 77.11275787677654, 0.77], [28.769807249187487, 77.11117798180635, 0.8], [28.769033832221353, 77.11628699281385, 0.71], [28.766299894596486, 77.11552782250351, 0.98], [28.771965620636482, 77.1165537283283, 0.86], [28.772001593813062, 77.11240906879613, 0.72], [28.769915167362598, 77.11671787326027, 0.79], [28.766084057721976, 77.11140368108781, 0.91], [28.768530212425183, 77.11226544198064, 0.86], [28.76826041576391, 77.11464554349419, 0.87], [28.767397067275922, 77.11128057238882, 0.89], [28.766965392503007, 77.1166152826778, 0.89], [28.763691862318232, 77.11050088396198, 0.94], [28.76509480342492, 77.11844139504593, 0.96], [28.770958380383764, 77.11727186240567, 0.91], [28.767900687517482, 77.1093929056712, 0.98], [28.764357360260213, 77.11111642745686, 0.99], [28.767918673583104, 77.11425569928075, 0.7], [28.764896952625204, 77.11905693854084, 0.73], [28.771497973699876, 77.11259373184457, 0.73], [28.769627384968402, 77.10982378611762, 0.98], [28.766803514994393, 77.11156782601977, 0.82], [28.76617398994594, 77.11056243831148, 0.76], [28.7641595092084, 77.1150764239406, 0.75], [28.765436545696996, 77.1176206703861, 0.89], [28.77013100481559, 77.11316823910646, 0.75], [28.771749783857725, 77.11115746368985, 0.97]]
vehicle=list()
for i in range(n):
    vehicle.append(swarmbot("127.0.0.1:" + str(14550 + i*10)))
for i in range(n):
    vehicle[i].arm_and_takeoff(5)
for i in range (n):
    loi=[finalwaypoints[i][0],finalwaypoints[i][1],5]
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
    veh["flag"]=1
    veh["payloaddroping"]=0
    uavs.append(veh)
extralist=[]
wstart=0.9
wend=0.4
T=180#(int(input("enter mopso time")))
timer=0
while True:
    for j in range(n):
        pos1=vehicle[j].get_pos()
        header=vehicle[j].vehicle.heading
        a1,a2,a3,a4=rectangle2(pos1,header,fol,fow)
        for i in loooopy:
            point = Point(i[0], i[1])
            polygon = Polygon([(a1[0],a1[1]), (a2[0],a2[1]), (a3[0],a3[1]), (a4[0],a4[1])])
            if polygon.contains(point)==True:
                if i[2]>uavs[j]["pbest"]:
                    uavs[j]["pbest"]=i[2]
                    uavs[j]["bestlocation"]=[i[0],i[1]]
                    #print(i)
                    #print(uavs[j])
                    x=minimumdistance(j)
                    #print(x)
                    updategbest(x)

                    for k in x:
                        v=[]
                        v=vehicle[k].vehicle.velocity
                        #print(v)
                        #print(uavs[k])
                        extralist.append(k)
                payload_drop(i)
                loooopy.remove(i)
                #print(len(loooopy))
        if uavs[j]["payloaddroping"]==1:
            checkifpayload(j)

    extralist=list(set(extralist))
    for jum in extralist:
        if uavs[jum]["payloaddroping"]==0:
            v=vehicle[jum].vehicle.velocity
            change_gpbest(jum)
            vel=generatevelocity(v,jum,timer,finalwaypoints)
            uavs[jum]["velocity"]=[vel[0],vel[1],vel[2]]
            vehicle[jum].update_vel(vel)
   


    if timer==180:
        print("end mopso")
        break
    timer=timer+1
    time.sleep(1)


