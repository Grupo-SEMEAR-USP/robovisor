#!/usr/bin/python

import time
import serial

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=10) 

number = 0
low_byte = number % 256
high_byte = number // 256

min = 256
max = 600
step_size = 500
count = 0
t = 0
flip = 1
res = 15

print(low_byte)
print(high_byte)
print(chr(low_byte))
print(chr(high_byte))
print(chr(low_byte).encode())
print(chr(high_byte).encode())

while True:

    if(count % step_size  == 0):
        t+=flip/res

    count+=1

    number = min + int((max - min) * t)

    low_byte = number % 256
    high_byte = number // 256

    if(number > max or number < min ):
        flip*=-1


    ser.write(chr(low_byte).encode())
    #time.sleep(100/1000000)
    ser.write(chr(high_byte).encode())
    #time.sleep(100/1000000)

    print(number)

ser.close()