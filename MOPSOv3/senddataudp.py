import socket
from pymavlink import mavutil
import time

mav = mavutil.mavlink_connection('udpin:127.0.0.1:14551')
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

data = ""
udpip1 = "10.60.48.97"
#udpip2 = "10.60.31.213"
udpip3 = "10.60.93.142"
udpip4 = "10.60.25.93"
port = 5002

while True:
	msg = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=10.0)
	lat = str(float(msg.lat)/10000000)
	lon = str(float(msg.lon)/10000000)

	msg = mav.recv_match(type='VFR_HUD', blocking=True, timeout=10.0)
	head = str(msg.heading)
	airsp = str(msg.airspeed)
	groundsp = str(msg.groundspeed)

	data = lat + " " + lon + " " + head + " " + airsp + " " + groundsp

	sock.sendto(data, (udpip1,port))
	#sock.sendto(data, (udpip2,port))
	sock.sendto(data, (udpip3,port))
	sock.sendto(data, (udpip4,port))
	print "sent"