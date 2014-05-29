#!/usr/bin/env python
import roslib
import rospy
import std_msgs
import fcntl
import os
import time
import signal
import sys

from std_msgs.msg import String
from subprocess import Popen, PIPE

class ProcessManager:
	def __init__(self):
		def handler(sig, f):
			rospy.loginfo("Caught shutdown signal")
			self.stop_all()
			sys.exit(0)

		self.processes = {}
		signal.signal(signal.SIGINT, handler)
		rospy.Subscriber("process_mgmt", String, lambda data: self.callback(data))

	def callback(self, data):
		command, arg = data.data.split()
		if command == "start":
			if arg == "stereo":
				self.start_stereo()
			elif arg == "all":
				self.start_all()
			else:
				self.start_normal(arg)
		elif command == "stop":
			if arg == "all":
				self.stop_all()
			else:
				self.stop_process(arg)
		elif command == "restart":
			if arg == "all":
				self.restart_all()
			else:
				self.restart_process(arg)
		elif command == "remove":
			self.remove(arg)

	def start_all(self):
		self.start_stereo()
		self.start_obstacle()

	def stop_all(self):
		for p in self.processes.values():
			if p.running():
				p.stop()
			else:
				print p.running()
				print p.proc.poll()
	
	def restart_all(self):
		for p in self.processes.values():
			p.restart()

	def start_process(self, args, name):
		if not name in self.processes:
			proc = Process(args, name)
			self.processes[name] = proc
		self.processes[name].start()

	def stop_process(self, name):
		if name in self.processes:
			self.processes[name].stop()

	def restart_process(self, name):
		if name in self.processes:
			self.processes[name].restart()

	def remove(self, name):
		if name in self.processes:
			proc = self.processes[name]
			if proc.running():
				proc.stop()
			del self.processes[name]

	def start_stereo(self):
		proc = StereoProcess()
		self.processes['stereo'] = proc
		proc.start()

	def start_normal(self, name):
		if name == "obstacle":
			args = ['rosrun', 'roscv2', 'obstacle_detect']
		self.start_process(args, name)


"""
	def start_pathfinding(self):
		move_args = ['rosrun', 'roscv', 'pathfinding']
		self.move_proc = Popen(move_args, stdin=PIPE, stdout=PIPE, stderr=PIPE)
		rospy.loginfo("Started pathfinding")
"""

class Process:
	def __init__(self, args, name=None, keep_alive=False):
		self.args = args
		self.proc = None
		self.name = name
		self.keep_alive = keep_alive
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
		time.sleep(1) #TODO
		self.start()

	def running(self):
		started = self.proc is not None
		running = started and self.proc.poll() is True
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
		if self.stereo_proc.running():
			self.stereo_proc.stop()
		if self.cam_proc.running():
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
	rospy.spin()
