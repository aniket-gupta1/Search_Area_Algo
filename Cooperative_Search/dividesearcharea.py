# import necessary functions
import dronekit
from pymavlink import mavutil
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
import numpy as np
from math import *
import threading
from scipy.spatial import Voronoi
from uncertainity_functions import contains_point


def distance(lat1,lat2,lon1,lon2): 
    """
    returns: distance between two coordinates (lat, lon)
    """

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

def bearing(lat1,lat2,lon1,lon2):
    """
    returns: bearing angle in degrees between 2 coordinates
    """

    lon1=radians(lon1)
    lon2=radians(lon2)
    lat1=radians(lat1)
    lat2=radians(lat2)
    dlon=abs(lon2-lon1)
    bearing=atan2(sin(dlon)*cos(lat2),cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(dlon))
    return (degrees(bearing))

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
    '''
    epsilon = 0.000001
    #we are giving angle in radians already
    rdistance = d / R

    lat2 = asin( sin(lat1) * cos(rdistance) + cos(lat1) * sin(rdistance) * cos(angle) )

    if cos(lat2) == 0 or abs(cos(lat2)) < epsilon: # Endpoint a pole
        lon2=lon1
    else:
        lon2 = ( (lon1 - asin( sin(angle)* sin(rdistance) / cos(lat2) ) + pi ) % (2*pi) ) - pi
        '''
    lat2 = asin(sin(lat1)*cos(ad) +cos(lat1)*sin(ad)*cos(angle))

    lon2 = lon1 + atan2(sin(angle)*sin(ad)*cos(lat1),cos(ad)-sin(lat1)*sin(lat2))
    

    lat = degrees(lat2)
    lon = degrees(lon2)
    return (lat, lon)

def rectangle(m,n,lat,lon,heading):
    #lat,lon are the mid points of an edge of rectangle
    #now we will first calculate mid point of opposite edge of the rectangle
    heading=float(heading)
    heading=radians(heading)
    elat,elon=pointRadialDistance(lat,lon,heading,n)
    rheading=heading+pi/2
    lheading=heading-pi/2
    lat1,lon1=pointRadialDistance(lat,lon,lheading,m/2)
    lat2,lon2=pointRadialDistance(lat,lon,rheading,m/2)
    lat3,lon3=pointRadialDistance(elat,elon,rheading,m/2)
    lat4,lon4=pointRadialDistance(elat,elon,lheading,m/2)
    p1=[lat1,lon1]
    p2=[lat2,lon2]
    p3=[lat3,lon3]
    p4=[lat4,lon4]
    return [p1, p2, p3, p4]

def coordinates(m,n,lat1,lon1,l,b,heading):
    """
    lat1, lon1: first vertice from left of rectangle
    m: length of rectangle
    n: breadth of rectangle
    l: cell length
    b: cell width
    heading: angle in degrees
    """
    no_of_fov_w=int(n/b)+1
    no_of_fov_l=int(m/l)+1
    wplist=[]
    heading=float(heading)
    heading=radians(heading)
    lat,lon=pointRadialDistance(lat1,lon1,heading,b/2)
    lat,lon=pointRadialDistance(lat,lon,heading+pi/2,l/2)
    p=[lat,lon]
    wplist.append(p)
    for i in range(no_of_fov_w-1):
        lat,lon=pointRadialDistance(lat,lon,heading,b)
        p=[lat,lon]
        wplist.append(p)

    for i in range(no_of_fov_l-1):
        lat,lon=pointRadialDistance(lat,lon,heading+pi/2,l)
        p=[lat,lon]
        wplist.append(p)
        for j in range(no_of_fov_w-1):
            if(i%2==0):
                lat,lon=pointRadialDistance(lat,lon,heading+pi,b)
                p=[lat,lon]
                wplist.append(p)
            else:
                lat,lon=pointRadialDistance(lat,lon,heading,b)
                p=[lat,lon]
                wplist.append(p)
    return wplist


# take observations
def takeobservations(fov_x,fov_y,lat,lon,heading, cell_width, cell_length, target_list, g_list, M):
    """
    """
    vertices = rectangle_mid_point(fov_x, fov_y, lat, lon, heading)
    """
    lat1 = vertices[0][0]
    lon1 = vertices[0][1]
    fov_g_list = coordinates(fov_x,fov_y,lat1,lon1,cell_width,cell_length,heading)
    """
    
    Z_list = np.zeros(M)
    for point in target_list:
        if contains_point(point,vertices):
            minindex = np.where(g_list == point)[0][0]
            print(minindex)
            Z_list[minindex] = 1

            #mindist = np.linalg.norm([g_list[0][0]-point[0], g_list[0][1]-point[1]])
            #minindex = 0
            #for i in range(M):
                #x = np.linalg.norm([g_list[i][0]-point[0], g_list[i][1]-point[1]])
                #if x < mindist:
                    #mindist = x
                    #minindex = i
            
    return Z_list

def rectangle_mid_point(m,n,lat,lon,heading):
    heading=float(heading)
    heading=radians(heading)
    elat1,elon1=pointRadialDistance(lat,lon,heading,n/2)
    elat2,elon2=pointRadialDistance(lat,lon,heading+pi,n/2)
    rheading=heading+pi/2
    lheading=heading-pi/2
    lat1,lon1=pointRadialDistance(elat2,elon2,lheading,m/2)
    lat2,lon2=pointRadialDistance(elat2,elon2,rheading,m/2)
    lat3,lon3=pointRadialDistance(elat1,elon1,rheading,m/2)
    lat4,lon4=pointRadialDistance(elat1,elon1,lheading,m/2)
    p1=[lat1,lon1]
    p2=[lat2,lon2]
    p3=[lat3,lon3]
    p4=[lat4,lon4]
    return p1,p2,p3,p4

    
