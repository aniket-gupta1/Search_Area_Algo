import matplotlib.pyplot as pl
import numpy as np
import scipy as sp
import scipy.spatial
import sys
import matplotlib.pyplot as plt
from dividesearcharea import rectangle_mid_point
from logging import debug

eps = sys.float_info.epsilon

bounding_box_helper = np.array(rectangle_mid_point(1000, 1000, 28.753300, 77.116118, 0))
x_min =  bounding_box_helper[0][0]
x_max = bounding_box_helper[2][0]
y_max = bounding_box_helper[1][1]
y_min = bounding_box_helper[0][1]
bounding_box = np.array([x_min,x_max,y_min,y_max])


def in_box(towers, bounding_box= bounding_box):
    return np.logical_and(np.logical_and(bounding_box[0] <= towers[:, 0],
                                         towers[:, 0] <= bounding_box[1]),
                          np.logical_and(bounding_box[2] <= towers[:, 1],
                                         towers[:, 1] <= bounding_box[3]))


def voronoi(towers, associated_point, bounding_box=bounding_box):
    # Select towers inside the bounding box
    i = in_box(towers, bounding_box)
    # Mirror points
    points_center = towers[i, :]
    points_left = np.copy(points_center)
    points_left[:, 0] = bounding_box[0] - (points_left[:, 0] - bounding_box[0])
    points_right = np.copy(points_center)
    points_right[:, 0] = bounding_box[1] + (bounding_box[1] - points_right[:, 0])
    points_down = np.copy(points_center)
    points_down[:, 1] = bounding_box[2] - (points_down[:, 1] - bounding_box[2])
    points_up = np.copy(points_center)
    points_up[:, 1] = bounding_box[3] + (bounding_box[3] - points_up[:, 1])
    points = np.append(points_center,
                       np.append(np.append(points_left,
                                           points_right,
                                           axis=0),
                                 np.append(points_down,
                                           points_up,
                                           axis=0),
                                 axis=0),
                       axis=0)
    # Compute Voronoi
    vor = sp.spatial.Voronoi(points)
    # Filter regions
    regions = []
    for region in vor.regions:
        flag = True
        for index in region:
            if index == -1:
                flag = False
                break
            else:
                x = vor.vertices[index, 0]
                y = vor.vertices[index, 1]
                if not(bounding_box[0] - eps <= x and x <= bounding_box[1] + eps and
                       bounding_box[2] - eps <= y and y <= bounding_box[3] + eps):
                    flag = False
                    break
        if region != [] and flag:
            regions.append(region)
    vor.filtered_points = points_center
    vor.filtered_regions = regions

    for i, reg in enumerate(vor.filtered_regions):

        #print ('Region:', i)
        #print ('Indices of vertices of Voronoi region:', reg)
        #print('Voronoi vertices of associated region:')
        vor_vertices = []
        for ind in reg:
            vor_vertices.append(vor.vertices[ind])
        #print ('Associated point:', arr[i], '\n') 
        if towers[i][0] == associated_point[0] and towers[i][1] == associated_point[1]:

            return (vor, vor_vertices)

    return (vor, None)
"""

def centroid_region(vertices):
    # Polygon's signed area
    A = 0
    # Centroid's x
    C_x = 0
    # Centroid's y
    C_y = 0
    for i in range(0, len(vertices) - 1):
        s = (vertices[i, 0] * vertices[i + 1, 1] - vertices[i + 1, 0] * vertices[i, 1])
        A = A + s
        C_x = C_x + (vertices[i, 0] + vertices[i + 1, 0]) * s
        C_y = C_y + (vertices[i, 1] + vertices[i + 1, 1]) * s
    A = 0.5 * A
    C_x = (1.0 / (6.0 * A)) * C_x
    C_y = (1.0 / (6.0 * A)) * C_y
    return np.array([[C_x, C_y]])


arr = np.array([[28.754142903188733, 77.11070344667375],[28.753153649343314, 77.11111376302556],[28.751175141672377, 77.11183181664124],[28.75207446299233, 77.11331921452077],[28.752928818034658, 77.11583240107137]])
vor = voronoi(arr, [28.753153649343314, 77.11111376302556])

exes = vor[0].vertices[:, 0]
ohs = vor[0].vertices[:, 1]
pts = []
for ex,oh in zip(exes, ohs):
    if (ex >= bounding_box[0] and ex <= bounding_box[1] and oh >= bounding_box[2] and oh <= bounding_box[3]):
        pts.append([ex, oh])

print(vor[1])

#print(pts)

for i, reg in enumerate(vor.filtered_regions):

    print ('Region:', i)
    print ('Indices of vertices of Voronoi region:', reg)
    print('Voronoi vertices of associated region:')
    for ind in reg:
        print(vor.vertices[ind], end = ",")
    print()
    print ('Associated point:', arr[i], )

fig = sp.spatial.voronoi_plot_2d(vor[0], show_vertices=True)
plt.show()
"""

