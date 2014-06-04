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

PROCESS_ORDER = ("camera", "stereo", "motor", "arm", "arm_state", "rover_state", "obstacle", "pathfinding", "find_base", "localization" )
PROCESS_ARGS = {"camera" : ['roslaunch', 'roscv', 'startCam.launch'],
"stereo": ['roslaunch', 'roscv', 'startStereo.launch'],
"motor": ['rosrun', 'roscv', 'rmotors.py'],
"arm": ['rosrun', 'roscv', 'arm_controller.py'],
"arm_state": ['rosrun', 'roscv', 'ArmStates.py'],
"rover_state": ['rosrun', 'roscv', 'state_machine.py'],
"obstacle": ['rosrun', 'roscv2', 'obstacle_detect'],
"pathfinding": ['rosrun', 'roscv2', 'pathfinding'],
"find_base": ['rosrun', 'roscv', 'find_base_station.py'],
"localization": ['rosrun', 'brain', 'local.py']
}

class ProcessManager:
	def __init__(self):
		def handler(sig, f):
			rospy.loginfo("Caught shutdown signal")
			self.stop_all()
			sys.exit(0)

		self.processes = {}
		for p in PROCESS_ORDER:
			self.processes[p] = Process(PROCESS_ARGS[p], p)

		signal.signal(signal.SIGINT, handler)
		rospy.Subscriber("process_mgmt", String, lambda data: self.callback(data))

	def callback(self, data):
		command, arg = data.data.split()
		if command == "start":
			if arg == "all":
				self.start_all()
			else:
				self.start_process(arg)
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

	def start_all(self):
		for p in PROCESS_ORDER:
			self.start_process(p)

	def stop_all(self):
		for p in reversed(PROCESS_ORDER):
			self.stop_process(p)
	
	def restart_all(self):
		self.stop_all()
		self.start_all()

	def start_process(self, name):
		if name in self.processes:
			self.processes[name].start()
		else:
			rospy.loginfo("Error - Can't start process `%s` - not present")

	def stop_process(self, name):
		if name in self.processes:
			self.processes[name].stop()
		else:
			rospy.loginfo("Error - Can't stop process `%s` - not present")

	def restart_process(self, name):
		if name in self.processes:
			self.processes[name].restart()
		else:
			rospy.loginfo("Error - Can't restart process `%s` - not present")

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

if __name__=="__main__":
	rospy.init_node("process_manage")
	proc = ProcessManager()
	proc.start_all()
	rospy.spin()
