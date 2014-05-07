#!/bin/bash
sudo avrdude -c stk500v2 -p atmega2560 -P /dev/ttyACM0 -F -U flash:w:MainDriveFirmware.hex
