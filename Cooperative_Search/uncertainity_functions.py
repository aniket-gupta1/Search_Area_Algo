from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import numpy as np 
from math import *
import time

def average_uncertainity(N, M, UAV_list, avg_uncertainity_deviation=0.4):
    """

    """
    
    avg = 0
    for i in range(N):
        for j in range(M):
            avg += abs(UAV_list[i].density[j]-avg_uncertainity_deviation)
    t2 = time.time()

    return avg


def contains_point(point, voro_points):
    """
    """
    polygon = Polygon(voro_points)
    return polygon.contains(Point(point[0], point[1]))

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


