import socket
from math import *
import time
import numpy as np
from math import *
import threading
from math import radians, cos, sin, asin, sqrt,atan2
import random
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from shapely.geometry import LineString
from time import time

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
    return(c*R*1000)
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
    dis=float(distance(p1[0],p1[1],p2[0],p2[1])/n)
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
    dis=float(distance(p1[0],p1[1],p2[0],p2[1])/n)
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

def create_uav_ports(address):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return sock

def create_and_bind_uav_ports(address):

    sock=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        sock.bind(address)
        sock.settimeout(1)
    except:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(address)
        sock.settimeout(1)
    return sock

def send_data(sock, address, port, data):
    sock.sendto(data, (address, port))

def recv_data(sock):
    try:
        data = sock.recv(1024)
        data = data.decode('utf-8')
        return data
    except:
        print("Exception occured in recv_data in helper functions file " + str(err))
        return None







"""
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
print(p1,p4)
futurepayload=[]
"""