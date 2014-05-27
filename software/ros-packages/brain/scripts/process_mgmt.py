#!/usr/bin/env python
import roslib
import rospy
import std_msgs
import fcntl
import os
import time
import signal
import sys

from subprocess import Popen, PIPE

class ProcessManager:
	def __init__(self):
		self.processes = {}

		def handler(sig, f):
			print "Got signal!"
			self.stop_all()
			sys.exit(0)

		signal.signal(signal.SIGINT, handler)

	def start_all(self):
		self.start_stereo()
		self.start_obstacle()

	def stop_all(self):
		for p in self.processes.values():
			p.stop()

	def start_process(self, args, name):
		proc = Process(args, name)
		self.processes[name] = proc
		proc.start()

	def stop_process(self, name):
		if name in self.processes:
			self.processes[name].stop()

	def start_stereo(self):
		proc = StereoProcess()
		self.processes['stereo system'] = proc
		proc.start()

	def start_obstacle(self):
		args = ['rosrun', 'roscv2', 'obstacle_detect']
		name = "obstacle detection"
		self.start_process(args, name)


"""
	def start_pathfinding(self):
		move_args = ['rosrun', 'roscv', 'pathfinding']
		self.move_proc = Popen(move_args, stdin=PIPE, stdout=PIPE, stderr=PIPE)
		rospy.loginfo("Started pathfinding")
"""

class Process:
	def __init__(self, args, name=None):
		self.args = args
		self.proc = None
		self.name = name
		if self.name is None:
			self.name = "%s" % self.args

	def start(self):
		if not self.running():
			rospy.loginfo("Starting process %s" % self.name)
			self.proc = Popen(self.args, stdout=PIPE, stderr=PIPE)

	def stop(self):
		if self.proc is not None:
			rospy.loginfo("Stopping process %s" % self.name)
			self.proc.terminate()
			self.proc.wait()

	def restart(self):
		rospy.loginfo("Restarting process %s" % self.name)
		self.stop()
		sleep(1) #TODO
		self.start()

	def running(self):
		started = self.proc is not None
		running = started and self.proc.poll()
		return running

class StereoProcess:
	def __init__(self):
		cam_args = ['roslaunch', 'roscv', 'startCam.launch']
		stereo_args = ['roslaunch', 'roscv', 'startStereo.launch']
		self.cam_proc = Process(cam_args, 'camera')
		self.stereo_proc = Process(stereo_args, 'stereo')

	def start(self):
		self.cam_proc.start()
		self.stereo_proc.start()

	def stop(self):
		self.stereo_proc.stop()
		self.cam_proc.stop()

	def restart(self):
		self.cam_proc.restart()
		self.stereo_proc.restart()

	def running(self):
		cam_running = self.cam_proc.running()
		stereo_running = self.stereo_proc.running()
		return cam_running and stereo_running

if __name__=="__main__":
	rospy.init_node("process_manage")
	proc = ProcessManager()
	proc.start_all()
	rospy.spin()
