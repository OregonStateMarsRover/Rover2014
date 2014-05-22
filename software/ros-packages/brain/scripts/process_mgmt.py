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
		self.cam_proc = Popen(cam_args, stdout=PIPE)
		sleep(1) #TODO
		stereo_args = ['roslaunch', 'roscv', 'startStereo.launch']
		self.stereo_proc = Popen(stereo_args, stdout=PIPE)

	def stop(self):
		self.proc_stop(self.stereo_proc)
		self.proc_stop(self.cam_proc)

	def proc_stop(self, proc):
		if proc is not None:
			proc.terminate()
			proc.wait()


	def restart(self):
		self.stop()
		sleep(1) #TODO
		self.start()

	def is_running(self):
		cam_running = self.proc_is_running(self.cam_proc)
		stereo_running = self.proc_is_running(self.stereo_proc)
		return cam_running and stereo_running

	def proc_is_running(self, proc):
		started = proc is not None
		running = started and proc.poll()
		return running



if __name__=="__main__":
	rospy.init_node("process_manage")
	proc = ProcessManager()
	proc.start_stereo()
	rospy.spin()
