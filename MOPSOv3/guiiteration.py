import os
import time
import socket
import pickle
import signal
import dronekit
import paramiko
import threading
import matplotlib
import numpy as np
import tkinter as Tk
import netifaces as ni
import PySimpleGUI as sg
import matplotlib.pyplot as plt
from gui  import graphic
from scp import SCPClient
from scipy.spatial import distance
from multiprocessing import Process
from dronekit import connect, VehicleMode

global graphic_object
graphic_object = graphic()

global params

def Launch_simulation_func(x):
	os.chdir("/home/aman/Swarm_Testing/")
	os.system("python3 simswarm.py " + str(x))

def RTL_func():
	Num_of_UAVs = int(params[0])

	for i in range(Num_of_UAVs):
		vehicle = connect('127.0.0.1:'+str(14552+i))
		time.sleep(0.5)
		vehicle.mode = VehicleMode("RTL")

def Plan_mission_func():
	os.chdir("/home/aman/ardupilot/ArduCopter")
	os.system("sim_vehicle.py -L L1 -I51 --map --sysid 51")

def Plan_formation_func():
	weight_matrix = graphic_object.display()
	dump_weight_matrix_file = open("weight_matrix", "wb")
	pickle.dump(weight_matrix, dump_weight_matrix_file)
	dump_weight_matrix_file.close()
	print("DONE baby")

def Save_wp_func():
	vehicle = connect('127.0.0.1:15060')
	print("Connected to planning vehicle")
	cmds=vehicle.commands
	cmds.download()
	cmds.wait_ready()

	waypoint_list=list()
	for cmd in cmds:
		waypoint_list.append([cmd.x,cmd.y])

	waypoint_list.append([1,1])

	wp_list_file = open("wp_list","wb")
	pickle.dump(waypoint_list, wp_list_file)
	wp_list_file.close()

def Start_mission_func():
	os.chdir("/home/aman/Swarm_Testing/")
	os.system("gnome-terminal -e 'python3 Launch.py "+str(params[0])+"'")

home_layout = [[sg.Text('SwarmSIM Planner', size=(22, 1), font=("Helvetica", 25), text_color='#033F63',justification='left'), sg.Image("logo_50.png")],
	[sg.Text('='  * 100, size=(80, 1), justification='center')],
	[sg.Text('Enter the Number of UAVs',size=(36,1),justification='right'), sg.InputText()],
	[sg.Text('Enter the UAV_ID that will remain in front',size=(36,1),justification='right'), sg.InputText()],
	[sg.Text('Enter the minimum distance between UAVs',size=(36,1),justification='right'), sg.InputText()],
	[sg.Text('_'  * 100, size=(80, 1), justification='center')],
	[sg.Button('Launch Simulation', button_color=('white', '#033F63')), sg.Button('Plan Mission', button_color=('white', '#033F63')),sg.Button('Plan Formation', button_color=('white', '#033F63')),sg.Button('Save WPs', button_color=('white', '#033F63')), sg.Button('RTL All UAVs', button_color=('white', '#EE6C4D'))],
	[sg.Text('_'  * 100, size=(80, 1), justification='center')],
	[sg.Button('Start Mission', button_color=('white', 'green'))]]

if __name__=="__main__":

	win = sg.Window('UAS-DTU SwarmSIM Planner', home_layout, auto_size_text=True, default_element_size=(40, 1))

	while True:
		try:
			event, values = win.Read()

			if values!=None:
				print(values)
				params = list(values.values())
				dump_params_file = open("params","wb")
				pickle.dump(params,dump_params_file)
				dump_params_file.close()

			if event==None:
				continue

			if event=="Launch Simulation":
				Ad_hoc_thread = threading.Thread(target = Launch_simulation_func, args = (params[0],))
				Ad_hoc_thread.start()

			elif event=="Plan Mission":
				Plan_mission_thread = threading.Thread(target = Plan_mission_func)
				Plan_mission_thread.start()

			elif event=="Plan Formation":
				Plan_formation_thread = threading.Thread(target = Plan_formation_func)
				Plan_formation_thread.start()

			elif event=="Save WPs":
				Save_Wp_thread = threading.Thread(target = Save_wp_func)
				Save_Wp_thread.start()

			elif event=="RTL All UAVs":
				RTL_thread = threading.Thread(target = RTL_func)
				RTL_thread.start()

			elif event=="Start Mission":
				start_mission_thread = threading.Thread(target = Start_mission_func)
				start_mission_thread.start()

		except Exception as err:
			print("fuck it crashed"+ str(err))
			event, values  = win.Read()