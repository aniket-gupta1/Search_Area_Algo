import matplotlib.pyplot as pl
import numpy as np
import scipy as sp
import scipy.spatial
import sys
import matplotlib.pyplot as plt
from dividesearcharea import rectangle_mid_point

eps = sys.float_info.epsilon
bounding_box_helper = np.array(rectangle_mid_point(1000, 1000, 28.753410, 77.116118, 0))
x_min = bounding_box_helper[0][0]
x_max = bounding_box_helper[2][0]
y_min = bounding_box_helper[0][1]
y_max = bounding_box_helper[1][1]
bounding_box = np.array([x_min,x_max,y_min,y_max])


def in_box(towers, bounding_box= bounding_box):
    return np.logical_and(np.logical_and(bounding_box[0] <= towers[:, 0],
                                         towers[:, 0] <= bounding_box[1]),
                          np.logical_and(bounding_box[2] <= towers[:, 1],
                                         towers[:, 1] <= bounding_box[3]))


def voronoi(towers, bounding_box=bounding_box):
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

    return vor

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


arr = np.array([[0, 0],
 [1, 1],
 [0, 2],
 [4, 3],
 [0, 5],])


#print(voronoi(arr,[0, 10, 0 , 10]))
"""
vor = (sp.spatial.Voronoi(arr))
print(vor.vertices)
fig = sp.spatial.voronoi_plot_2d(vor)

plt.show()


[[0.74298619 0.29963513]
 [0.14033699 0.59071851]
 [0.32851465 0.72294695]
 [0.57220957 0.53412554]
 [0.3972714  1.04243969]
 [0.73324051 0.8747537 ]
 [0.5805757  0.01463616]
 [0.24867535 0.38369567]
 [1.0268243  0.39926251]
 [1.88700073 0.60429849]
 [0.75800064 0.74872197]
 [0.76292326 0.76783348]]

"""
bounding_box = [-1,5,-1,5]
vor = voronoi(arr, bounding_box)
exes = vor.vertices[:, 0]
ohs = vor.vertices[:, 1]
pts = []
for ex,oh in zip(exes, ohs):
    if (ex >= bounding_box[0] and ex <= bounding_box[1] and oh >= bounding_box[2] and oh <= bounding_box[3]):
        pts.append([ex, oh])

print(vor.vertices[0])
print(len(vor.vertices))
print(pts)
fig = sp.spatial.voronoi_plot_2d(vor, show_vertices=True)


plt.show()

