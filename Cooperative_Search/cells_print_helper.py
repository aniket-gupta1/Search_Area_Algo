from dividesearcharea import rectangle_mid_point, coordinates



def write(m, n, lat, lon, heading):
	p = rectangle_mid_point(m, n, lat, lon, heading)
	cell_list = coordinates(m, n, p[0][0], p[0][1], 10, 10, heading)
	with open("/home/abhinavjava/cells.txt", "w") as cells:
		for coordinate in cell_list:
			print(str(coordinate[0]) + " " + str(coordinate[1]) + " " + str(1.5), file=cells)

		print(str(28.75414290238) + " " + str(77.11490918927977) + " " + str(3), file=cells)
		print(str(28.753738207647647) + " " + str(77.11496047992799) + " " + str(3), file=cells)
		print(str(28.751714734261203) + " " + str(77.11378081931231) + " " + str(3), file=cells)
		print(str(28.74996105729723) + " " + str(77.11290889816897) + " " + str(3), file=cells)
		print(str(28.75144493809498) + " " + str(77.1121395539051) + " " + str(3), file=cells)

	cells.close()




write(1000, 1000, 28.754143,77.115781,0)












