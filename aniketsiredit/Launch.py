import os
import sys
import dronekit
import getpass
from time import sleep

N = int(sys.argv[1])
launch_string_mopso="gnome-terminal"

"""
for i in range(N):
	os.chdir("/home/"+getpass.getuser()+"/Swarm_Testing/")
	launch_string += " --tab -e 'python3 modified_server.py " + str(i) +"'"
	sleep(0.1)
os.system(launch_string)

sleep(3)
"""

for i in range(N):
	os.chdir("/home/"+getpass.getuser()+"/Swarm_Testing/aniketsiredit")
	launch_string_mopso += " --tab -e 'python3 mopso.py " + str(i) +"'"
	sleep(0.1)

os.system(launch_string_mopso)