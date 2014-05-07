"""
Program to interface with the 2014 Mars Rover Arm system.
Nick McComb - 2014 

INPUT:  A controller that is controlled using pygame / a Tkinter GUI
OUTPUT: Sends a packet over serial to an arduino

TODO:
-Implement command byte

ARDUINO TODO:
-Implement e-stop

Requied to use:
-Tkinter
-ttk
-serial
-time
-math
-pygame

-xboxdrv goo.gl/CU5qNq
"""

from Tkinter import *
from ttk import Button, Style

import serial
import time
import math
import sys
import pygame
from pygame.locals import *
from listPorts import * #REQUIRES SUPPORTING FILE

errorDelay = 3

if getValidPort() == 'Error':
	print 'No valid serial device connected. Exiting program...'
	time.sleep(errorDelay)
	quit()
ser = serial.Serial(getValidPort(), 9600, timeout=.05)  #DOES VARY BY PLATFORM!
ser.flushInput()
ser.read(100)


pygame.init()
pygame.joystick.init()
if pygame.joystick.get_count() == 0:
	print 'No valid controller detected. Exiting program...'
	time.sleep(errorDelay)
	quit()
#if pygame.joystick.get_count() > 1:
#	print 'Too many controllers detected. Please restart the Virtual Machine. Exiting program...'
#	time.sleep(errorDelay)
#	quit()
yJoystick = pygame.joystick.Joystick(pygame.joystick.get_count()-1) #Can't 
yJoystick.init()

print "Initialized Joystick: %s" % yJoystick.get_name()
print "Number of Axes: ",
print yJoystick.get_numaxes()
print "ID: ",
print "HI"

#Globals

limitingDelay = .1  #limits the delay between incriment and decrement for both actuators
actResolution = .05 #Determines the 'step' for controlling the actuator
#Variables used for the first actuator
startTimeAct1 = 0 #Used for the start time for delays
canSetAct1 = 1
act1SoftLower = .2 #Soft bounds in the GUI for the actuator
act1SoftUpper = 3.75
#Variables used for the second actuator
startTimeAct2 = 0
canSetAct2 = 1
act2SoftLower = 0.05 #Soft bounds in the GUI for the actuator
act2SoftUpper = 3.6
global act1
global act2
#Variables used for the base
global baseRot
baseRotSoftLower = 30
baseRotSoftUpper = 330
baseRotResolution = 5 #Determines the 'step' for controlling the base rotation
canSetBaseRot = 1
startTimeBaseRot = 0
#Variables used for the grip
#GRIP OPERATION: R1 (b5) to grip, L1 (b4) to release
grip = 0
#Packet-related information
canSendPacket = 1
packetDelay = 1 #Delay between 2 packets being sent in SECONDS
packetSendTime = 0  #Inital time that a packet is sent

#Increments the counter for Actuator 1
def incrementAct1():
	global canSetAct1
	global startTimeAct1
	global limitingDelay
	global act1
	global act1SoftUpper
	global actResolution
	if((not canSetAct1) and (abs(time.time()-startTimeAct1) > limitingDelay)):  #Number of seconds to delay repeating input
		canSetAct1 = 1
	if(canSetAct1):
		currentStroke = act1.get()
		if (currentStroke +actResolution)<= act1SoftUpper:
			act1.set(act1.get()+actResolution)
		startTimeAct1 = time.time()
		canSetAct1 = 0

#Decrements the counter for Actuator 1	
def decrementAct1():
	global canSetAct1
	global startTimeAct1
	global limitingDelay
	global act1
	global act1SoftLower
	global actResolution
	if((not canSetAct1) and (abs(time.time()-startTimeAct1) > limitingDelay)):  #Number of seconds to delay repeating input
		canSetAct1 = 1
	if(canSetAct1):
		currentStroke = act1.get()
		if (currentStroke -actResolution) >= act1SoftLower:
			act1.set(act1.get()-actResolution)
		startTimeAct1 = time.time()
		canSetAct1 = 0

#Increments the counter for Actuator 2
def incrementAct2():
	global canSetAct2
	global startTimeAct2
	global limitingDelay
	global act2
	global act2SoftUpper
	global actResolution
	if((not canSetAct2) and (abs(time.time()-startTimeAct2) > limitingDelay)):  #Number of seconds to delay repeating input
		canSetAct2 = 1
	if(canSetAct2):
		currentStroke = act2.get()
		if (currentStroke +actResolution) <= act2SoftUpper:
			act2.set(act2.get()+actResolution)
		startTimeAct2 = time.time()
		canSetAct2 = 0

#Decrements the counter for Actuator 2
def decrementAct2():
	global canSetAct2
	global startTimeAct2
	global limitingDelay
	global act2
	global act2SoftLower
	global actResolution
	if((not canSetAct2) and (abs(time.time()-startTimeAct2) > limitingDelay)):  #Number of seconds to delay repeating input
		canSetAct2 = 1
	if(canSetAct2):
		currentStroke = act2.get()
		if (currentStroke -actResolution) >= act2SoftLower:
			act2.set(act2.get()-actResolution)
		startTimeAct2 = time.time()
		canSetAct2 = 0

#Increments the counter for the Base Rotation 
def incrementBaseRot():
	global baseRot
	global baseRotSoftUpper
	global limitingDelay
	global canSetBaseRot
	global startTimeBaseRot
	global baseRotResolution
	if((not canSetBaseRot) and (abs(time.time()-startTimeBaseRot) > limitingDelay)):
		canSetBaseRot = 1
	if(canSetBaseRot):
		currentRot = baseRot.get()
		if(currentRot + baseRotResolution) <= baseRotSoftUpper:
			baseRot.set(baseRot.get() + baseRotResolution)
		startTimeBaseRot = time.time()
		canSetBaseRot = 0
		
#Decrements the counter for the Base Rotation
def decrementBaseRot():
	global baseRot
	global baseRotSoftLower
	global limitingDelay
	global canSetBaseRot
	global startTimeBaseRot
	global baseRotResolution
	if((not canSetBaseRot) and (abs(time.time()-startTimeBaseRot) > limitingDelay)):
		canSetBaseRot = 1
	if(canSetBaseRot):
		currentRot = baseRot.get()
		if(currentRot - baseRotResolution) >= baseRotSoftLower:
			baseRot.set(baseRot.get() - baseRotResolution)
		startTimeBaseRot = time.time()
		canSetBaseRot = 0
	

#This function checks the variable 'canSendPacket' and if it determines that it can send a packet,
#then it invokes the sendSerial() helepr function
def sendPacket():
	global canSendPacket
	global packetDelay
	global packetSendTime
	global act1Label

	#Hooray for redundant code!
	if (not canSendPacket):
		act1Label.set("Too soon!")
	if canSendPacket:
		act1Label.set("Sent Packet")
		sendSerial()
		canSendPacket = 0
	else:
		print "Too soon!"
	
	
#Function that is used by sendPacket() to actually send the packet over pyserial
#Uses the serial connection that was already open, ser
#HELPER FUNCTION
def sendSerial():
	global act1
	global act2
	global grip
	global baseRot
	global ser  #Serial connection
	
	#Setup
	act1Temp = int(act1.get() * 100)
	act2Temp = int(act2.get() * 100)
	baseRotTemp = int(baseRot.get())
	
	header = 255
	command = 0 #NEED TO IMPLEMENT COMMAND BYTE
	if grip:
		command = command + 2;  #Set the second bit in binary
	baseRotVal1 = 0  #BASE ROTATION NOT YET IMPLEMENTED
	baseRotVal2 = 0
	if act1Temp < 255:
		act1Val1 = act1Temp 
		act1Val2 = 0
	else:
		act1Val1 = 255
		act1Val2 = act1Temp % 255
	if act2Temp < 255:
		act2Val1 = act2Temp
		act2Val2 = 0
	else:
		act2Val1 = 255
		act2Val2 = act2Temp % 255
	if baseRotTemp < 255:
		baseRotVal1 = baseRotTemp
		baseRotVal2 = 0
	else:
		baseRotVal1 = 255
		baseRotVal2 = baseRotTemp % 255
	
	checksum = calcChecksum(command, baseRotVal1, baseRotVal2, act1Val1, act1Val2, act2Val1, act2Val2)
	tail = 255
	
	
	
	#Write the calculated bytes to serial
	ser.write(chr(header))
	ser.write(chr(command))
	ser.write(chr(baseRotVal1))
	ser.write(chr(baseRotVal2))
	ser.write(chr(act1Val1))
	ser.write(chr(act1Val2))
	ser.write(chr(act2Val1))
	ser.write(chr(act2Val2))
	ser.write(chr(checksum))
	ser.write(chr(tail))

def calcChecksum(command, baseRotVal1, baseRotVal2, act1Val1, act1Val2, act2Val1, act2Val2):
	return command ^ baseRotVal1 ^ baseRotVal2 ^ act1Val1 ^ act1Val2 ^ act2Val1 ^ act2Val2

def writeOne():
	ser.write('1')
	
def writeZero():
	ser.write('0')
	

class Example(Frame):
	#print(list(serial_ports()))
	def __init__(self, parent):
		Frame.__init__(self, parent)
		
		self.parent = parent
		
		self.initUI()
		
	def initUI(self):
		
		self.parent.title("Serial GUI")
		self.style = Style()
		self.style.theme_use("default")
		
		self.pack(fill=BOTH, expand = 1)
		
		quitButton = Button(self, text="Quit",
			command = self.quit)
		quitButton.place(x=300, y=350)

	def task(self):
		pygame.event.get()
		#left = int((yJoystick.get_axis(1)* -127) + 127)
		print "			",
		leftY = yJoystick.get_axis(1)
		leftY *= -1;
		#print "JoystickX value",  #Debugging
		#print leftY  #Debugging
		if leftY > .5:  #When the joystick reads more than .5 either way, inc or dec
			incrementAct1()
		elif leftY < -.5:
			decrementAct1()
		print "			",
		rightY = yJoystick.get_axis(3)
		rightY *= -1
		if rightY > .5: #Increment
			incrementAct2()
		elif rightY < -.5:
			decrementAct2()
		global act1
		global act2
		#act1.set(1) #debug
		#act2.set(2) #debug
		print "Actuator 1 Value: ",
		print act1.get()
		print "Actuator 2 Value: ",
		print act2.get()
		global grip
		r1 = yJoystick.get_button(5)
		l1 = yJoystick.get_button(4)
		if r1:
			grip = 1
		if l1:
			grip = 0
		if yJoystick.get_button(2):
			decrementBaseRot()
		if yJoystick.get_button(1):
			incrementBaseRot()
		global canSendPacket
		global act1Label
		if ser.read() == 'r':
			canSendPacket = 1
			act1Label.set("Recieved Response")

		xButton = yJoystick.get_button(0)
		if xButton:
			sendPacket()
		self.update()
		self.after(0,self.task) # Reschedule ASAP


#This main function should only run once (essentially) because it calls another infinitly looping function,
#app.task(), which calls itself. This function is maily used for initialization of the GUI.
def main():
	global actResolution
	root = Tk()
	root.geometry("800x400+100+100")
	app = Example(root)
	#Create the scale for actuator 1
	global act1Var
	act1Var = 2.0
	global act1 
	act1 = Scale(root, from_= 0, to = 3.75, tickinterval=.5, width=40, resolution=actResolution, sliderlength=15, length=200, label="Lower Actuator")
	act1.set(act1Var)
	act1.place(x=25, y=100)
	#Creat the scale for actuator 2
	global act2Var
	act2Var = 2.0
	global act2
	act2 = Scale(root, from_=0, to = 3.75, tickinterval=.5, width=40, resolution=actResolution, sliderlength=15, length=200, label="Upper Actuator")
	act2.set(act2Var)
	act2.place(x=275, y=100)
	global baseRotVar
	baseRotVar = 180
	global baseRot
	baseRot = Scale(root, from_=10, to = 370, tickinterval=45, width=40, resolution=1, sliderlength=15, length=200, label="Base Angle")
	baseRot.set(baseRotVar)
	baseRot.place(x=500, y=100)
	
	#Text
	global act1Label
	act1Label = StringVar()
	Label(root, textvariable=act1Label).pack()#place(x=25, y=400)
	act1Label.set("Serial Interface GUI")

	
	root.after(1500,app.task)
	
	root.mainloop()
	
if __name__ == '__main__':
	main()

#Closes the serial connection
ser.close()

print "Closed Serial Connection"
