from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import numpy as np 

def average_uncertainity(N, M, UAV_list, avg_uncertainity_deviation=1):
	"""

	"""
	avg = 0
	for i in range(N):
		for j in range(M):
			avg += np.linalg.norm(UAV_list[[i][j][0]-avg_uncertainity_deviation, UAV_list.density[i][j][1] - avg_uncertainity_deviation])

	return avg

def contains_point(point, voro_points):
	"""
	"""
	polygon = Polygon(voro_points)
	return polygon.contains(Point(point[0], point[1]))


