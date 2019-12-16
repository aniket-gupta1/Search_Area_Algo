import os
import sys
import dronekit
import getpass
from time import sleep

N = int(sys.argv[1])

os.system("gnome-terminal --tab -e 'python3' --tab -e 'python'")
sleep(0.1)