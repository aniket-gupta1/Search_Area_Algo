import numpy as np
with open('/home/abhinavjava/cells.txt','r') as cells:
    mylines = np.array(cells.readlines())
    print(mylines)
    lat = mylines[:, 0]
    lon = mylines[:, 1]
    minlat = min(lat)
    maxlat = max(lat)
    minlon = min(lon)
    maxlon = max(lon)

print(minlat, maxlat, minlon, maxlon)
