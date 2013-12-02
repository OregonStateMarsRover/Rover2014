import os
import pygame
import time
from pygame.locals import *


if __name__ == '__main__':
  pygame.init()
  pygame.joystick.init()
  MyJoystick = pygame.joystick.Joystick(0)
  MyJoystick.init()
  
  while(1):
    os.system('clear')
    pygame.event.get()
    #for i in range(0, MyJoystick.get_numaxes()):
    print "Left Value " + str(MyJoystick.get_axis(1))
    print "Right Value " + str(MyJoystick.get_axis(4))
    time.sleep(.001)