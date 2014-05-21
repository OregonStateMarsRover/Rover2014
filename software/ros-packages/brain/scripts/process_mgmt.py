#!/usr/bin/env python
import roslib
import rospy
import std_msgs
import fcntl
import os
import time

from subprocess import Popen, PIPE

def nb_read(f):
	fd = f.fileno()
	fl = fcntl.fcntl(fd, fcntl.F_GETFL)
	fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)
	try:
		return f.read()
	except:
		return ""

class ProcessManager:
	def __init__(self):
		self.cam_proc = None
		self.stereo_proc = None
		self.obs_proc = None
		self.move_proc = None

	def start_stereo(self):
		sp = StereoProcess()
		sp.start()

"""
	def start_pathfinding(self):
		move_args = ['rosrun', 'roscv', 'pathfinding']
		self.move_proc = Popen(move_args, stdin=PIPE, stdout=PIPE, stderr=PIPE)
		rospy.loginfo("Started pathfinding")
"""

class StereoProcess:
	def __init__(self):
		self.cam_args = ['roslaunch', 'roscv', 'startCam.launch']
		self.stero_args = ['roslaunch', 'roscv', 'startStereo.launch']
		self.cam_running = False
		self.stereo_running = False

	def start(self):
		rospy.loginfo("Starting stereo system...")
		cam_args = ['roslaunch', 'roscv', 'startCam.launch']
		self.cam_proc = Popen(cam_args, stdin=PIPE, stdout=PIPE, stderr=PIPE)
		stereo_args = ['roslaunch', 'roscv', 'startStereo.launch']
		self.stereo_proc = Popen(stereo_args, stdin=PIPE, stdout=PIPE, stderr=PIPE)
		#Read from stereo_proc.stdout.readlines() ????
		while True:
			time.sleep(1)
			print "xxx"
			sr = nb_read(self.stereo_proc.stdout)
			if sr != "":
				print sr


if __name__=="__main__":
	rospy.init_node("process_manage")
	proc = ProcessManager()
	proc.start_stereo()
	rospy.spin()
