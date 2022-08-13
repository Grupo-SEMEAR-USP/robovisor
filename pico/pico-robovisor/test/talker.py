#!/usr/bin/python

import time
import serial

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=10) 

number = 630
low_byte = number % 256
high_byte = number // 256

while True:
    ser.write(chr(low_byte).encode())
    #time.sleep(100/1000000)
    ser.write(chr(high_byte).encode())
    #time.sleep(100/1000000)

ser.close()