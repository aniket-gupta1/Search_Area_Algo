#!/usr/bin/env python
'''
cell Module
'''

import os
import os.path
import sys
from pymavlink import mavutil
import errno
import time
import random

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.mavproxy_map import mp_slipmap

#sys.path.append('/home/jbhowmick/interop/client')
#import interop

class obstacle(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(obstacle, self).__init__(mpstate, "obstacle", "")
        self.status_callcount = 0
        self.boredom_interval = 0.1 # seconds
        self.last_bored = time.time()

        self.packets_mytarget = 0
        self.packets_othertarget = 0
        self.verbose = False

        self.obstacle_settings = mp_settings.MPSettings(
            [ ('verbose', bool, False),
          ])
        self.add_command('obstacle', self.cmd_obstacle, "obstacle module", ['status','set (LOGSETTING)'])

	'''self.client = interop.Client(url='http://localhost:8000',
                        username='testuser',
                        password='testpass')'''


    def usage(self):
        '''show help on command line options'''
        return "Usage: obstacle <status|set>"

    def cmd_obstacle(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print(self.usage())
        elif args[0] == "status":
            print(self.status())
        elif args[0] == "set":
            self.obstacle_settings.command(args[1:])
        else:
            print(self.usage())
        self.mpstate.map.add_object(mp_slipmap.SlipClearLayer('Obstacle'))

        #missions = client.get_missions()
        #print missions

        #stationary_obstacles, moving_obstacles = self.client.get_obstacles()
        #s_o=stationary_obstacles
        #print m_o
        #print s_o[0].latitude
        #print s_o[0].longitude
        #print s_o[0].cylinder_radius
        obstaclefile = open("/home/abhinavjava/cells.txt", "r")
        print "Before slipcircle"

        lines=obstaclefile.readlines()
        for i in lines:
            thisline=i.split(" ")
            location=(float(thisline[0]),float(thisline[1]))
            r=float(thisline[2])
            if r == 3:
                self.mpstate.map.add_object(mp_slipmap.SlipCircle('Obstacle Circle'+i, layer='Obstacle', latlon=location, radius=r, linewidth=-1, color=(0,255,0)))
            elif r == 5:
                self.mpstate.map.add_object(mp_slipmap.SlipCircle('Obstacle Circle'+i, layer='Obstacle', latlon=location, radius=r, linewidth=-1, color=(255,0,0)))
        print "After slipcircle"


	"""while True:    #dynamic obstacles
		stationary_obstacles, moving_obstacles = self.client.get_obstacles()
		m_o=moving_obstacles
		#print m_o
		j=0
		for j in range(len(m_o)):
			loc=(m_o[j].latitude,m_o[j].longitude)
			rad=(float(m_o[j].sphere_radius)*0.3048)
			self.mpstate.map.add_object(mp_slipmap.SlipCircle('m_Obstacle Circle'+str(j), layer='Obstacle', latlon=loc, radius=rad, linewidth=-1, color=(255,0,0)))

		#time.sleep(0.1)
	"""

    def status(self):
        '''returns information about module'''
        self.status_callcount += 1
        self.last_bored = time.time() # status entertains us
        return("status called %(status_callcount)d times.  My target positions=%(packets_mytarget)u  Other target positions=%(packets_mytarget)u" %
               {"status_callcount": self.status_callcount,
                "packets_mytarget": self.packets_mytarget,
                "packets_othertarget": self.packets_othertarget,
               })

    '''def boredom_message(self):
        self.mpstate.map.add_object(mp_slipmap.SlipClearLayer('Obstacle'))
	self.mpstate.map.add_object(mp_slipmap.SlipClearLayer('m_Obstacle'))

	stationary_obstacles, moving_obstacles = self.client.get_obstacles()
	s_o=stationary_obstacles

	i=0   #static obstacles
	for i in range(len(s_o)):
		location=(s_o[i].latitude,s_o[i].longitude)
		#print location
		r=(float(s_o[i].cylinder_radius)*0.3048)
		#print r
		self.mpstate.map.add_object(mp_slipmap.SlipCircle('Obstacle Circle'+str(i), layer='Obstacle', latlon=location, radius=r, linewidth=-1, color=(255,0,0)))
		i+=1

	stationary_obstacles, moving_obstacles = self.client.get_obstacles()
	m_o=moving_obstacles
	#print m_o
	j=0
	for j in range(len(m_o)):
		loc=(m_o[j].latitude,m_o[j].longitude)
		rad=(float(m_o[j].sphere_radius)*0.3048)
		self.mpstate.map.add_object(mp_slipmap.SlipCircle('m_Obstacle Circle'+str(j), layer='m_Obstacle', latlon=loc, radius=rad, linewidth=-1, color=(255,0,0)))

	time.sleep(0.1)'''



    def idle_task(self):
        '''called rapidly by mavproxy'''
        now = time.time()
        if now-self.last_bored > self.boredom_interval:
            self.last_bored = now
            message = self.boredom_message()
            self.say("%s: %s" % (self.name,message))
            # See if whatever we're connected to would like to play:
            self.master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE,
                                            message)

    def mavlink_packet(self, m):
        '''handle mavlink packets'''
        if m.get_type() == 'GLOBAL_POSITION_INT':
            if self.settings.target_system == 0 or self.settings.target_system == m.get_srcSystem():
                self.packets_mytarget += 1
            else:
                self.packets_othertarget += 1

def init(mpstate):
    '''initialise module'''
    return obstacle(mpstate)
