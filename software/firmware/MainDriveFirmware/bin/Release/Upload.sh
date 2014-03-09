#!/bin/bash
sudo /home/corwin/arduino-1.5.6/hardware/tools/avrdude -C/home/corwin/arduino-1.5.6/hardware/tools/avrdude.conf -patmega2560 -cwiring -P /dev/ttyACM0 -b115200 -D -U flash:w:MainDriveFirmware.hex:i
