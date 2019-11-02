from dividesearcharea import rectangle_mid_point, coordinates



def write(m, n, lat, lon, heading):
	p = rectangle_mid_point(m, n, lat, lon, heading)
	cell_list = coordinates(m, n, p[0][0], p[0][1], 10, 10, heading)
	with open("/home/abhinavjava/cells.txt", "w") as cells:
		for coordinate in cell_list:
			print(str(coordinate[0]) + " " + str(coordinate[1]) + " " + str(3), file=cells)

		print(str(28.750050989881974) + " " + str(77.11070344667375) + " " + str(5), file=cells)
		print(str(28.751355006210480) + " " + str(77.11070344667370) + " " + str(5), file=cells)
		print(str(28.751175141544177) + " " + str(77.11249858181716) + " " + str(5), file=cells)
		print(str(28.754142902626818) + " " + str(77.11362695178462) + " " + str(5), file=cells)
		print(str(28.751444937641303) + " " + str(77.11449887292797) + " " + str(5), file=cells)

	cells.close()




write(1000, 1000, 28.754143,77.115781,0)












