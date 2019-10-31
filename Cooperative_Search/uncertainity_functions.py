from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import numpy as np 

def average_uncertainity(N, M, density_list, avg_uncertainity_deviation):
	"""

	"""
	avg = 0
	for i in range(N):
		for j in range(M):
			avg += np.linalg.norm([density_list[i][0]-avg_uncertainity_deviation, density_list[i][1] - avg_uncertainity_deviation])

	return avg

def contains_point(point, voro_points):
	"""
	"""
	polygon = Polygon(voro_points)
	return polygon.contains(Point(point[0], point[1]))


