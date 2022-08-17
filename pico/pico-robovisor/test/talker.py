#!/usr/bin/python

import time
import serial

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=10) 

number = 254

while True:
    ser.write(chr(number).encode())
    print(chr(number).encode())

ser.close()