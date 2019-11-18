from scipy.spatial import Voronoi
import numpy as np 
import math
import time
from threading import *
import sys
from GraphClass import Graph
import itertools
from Swarm_Class import UAVSwarmBot
import dronekit
from dividesearcharea import coordinates, takeobservations, rectangle_mid_point
from uncertainity_functions import average_uncertainity, contains_point, initial_wps_func, final_wps_func
from main import colab_search
from generate_voronoi import voronoi
import logging


m = 1000
n = 1000
N=10
logging.basicConfig(level=logging.DEBUG)
centre_wp = [28.753300, 77.116118] # enter coordinate

# make uav list and arm and take off
# should return a list of centre coordinates
target_list = np.array([np.array([28.755413308079383, 77.1133998428576]), np.array([28.756762289717177, 77.11545145223369]),
np.array([28.754603915780642, 77.12099079754887]), np.array([28.75460391948231, 77.1111430725439]),np.array([28.752355612035693, 77.12027272985]), np.array([28.752895205307713, 77.11945208609963]),
np.array([28.748848260202106, 77.11381016473283 ]), np.array([28.750556971253307, 77.11381016473283]),np.array([28.75037710450266, 77.12027272985]), np.array([28.751905952466625, 77.11699015484842]),
np.array([28.75316500283068, 77.11668241785934]), np.array([28.757122019092186, 77.11350241890909]),np.array([28.753704597143887, 77.11309209703388]), np.array([28.75568310348152, 77.11627209598412]),
np.array([28.75073683449474, 77.11668241785934]), np.array([28.75001737613025, 77.11955467098574]),np.array([28.756402561845857, 77.1133998428576]), np.array([28.75505357739327, 77.11883660328687]),
np.array([28.754334122807787, 77.11165597047062]), np.array([28.751995884395786, 77.11760563766124]),np.array([28.75208581601658, 77.11904176422445]), np.array([28.75757167916251, 77.11545145223369]),
np.array([28.750916697967458, 77.11893918817296]), np.array([28.75352473147304, 77.11668241785934]),np.array([28.753075069166247, 77.12068305172518]), np.array([28.756492492001193, 77.11873402723538]),
np.array([28.757122019786184, 77.11165597047062]), np.array([28.756582426012844, 77.11381016473283]),np.array([28.75631262983952, 77.1129895209824]), np.array([28.751546224788488, 77.11442564754564]),
np.array([28.75640255960931, 77.11934951004814]), np.array([28.75334486688188, 77.11740047672363 ]),np.array([28.75136636027445, 77.11493854547234]), np.array([28.757301884300027, 77.1111430725439]),
np.array([28.755862966722887, 77.11914434911056 ]), np.array([28.75316500148104, 77.12027272985]),np.array([28.7539743939341, 77.11227145328346]), np.array([28.754513985278184, 77.11657983297322]),
np.array([28.75118649741841, 77.11104048765779]), np.array([28.757122017973817, 77.11647725692173])])

p = rectangle_mid_point(1000, 1000, 28.753300, 77.116118, 0)
cell_list = coordinates(m, n, p[0][0], p[0][1], 10, 10, 0)
temp = initial_wps_func(N, 28.749498, 77.111593, 0, 900, 900)

points = []
for pts in temp:
    pts = dronekit.LocationGlobalRelative(pts[0], pts[1], 35)
    points.append(pts)


UAVlocdict = dict()
vehicle = list() # list of different thread UAVs
for i in range(N):
    UAVthread = UAVSwarmBot("127.0.0.1:" + str(14550 + i*10))
    UAVthread.start()
    vehicle.append(UAVthread)
    UAVlocdict[i] = UAVthread
    UAVthread.g_list = cell_list
    UAVthread.UAVID += i
    UAVthread.arm_and_takeoff(5)
    UAVthread.vehicle.simple_goto(points[i])
    time.sleep(1)

M = len(cell_list)
logging.debug("Starting search...")
time.sleep(20)
while True:
    if average_uncertainity(N, M, UAVlocdict) <= 0.005:
        break

    for i in range(N):
        colab_search(vehicle[i], N, M, 100, cell_list, UAVlocdict)

for i in range(N):
    vehicle[i].join()

for UAV2 in UAVlocdict:
    logging.debug("UAV: ", str(UAVlocdict[UAV2].getlocation()))