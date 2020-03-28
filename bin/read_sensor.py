#!/usr/bin/python

# pip install pyserial
# Update serial port below to correct value
# ./read_sensor.py > data.txt

import datetime
import serial

ser = serial.Serial('/dev/cu.usbserial-AL05MEZR', baudrate=115200, timeout=None, xonxoff=True, rtscts=False, dsrdtr=False)

try:
  while True:
    data = ser.readline()
    print str(datetime.datetime.now().time()) + ': ' + data
finally:
  ser.close()
