import numpy as np
import serial
import time

isWindowsOS = True

if isWindowsOS:
    port = 'COM4'
else:
    port = '/dev/ttyUSB0'

ser = serial.Serial(port=port, baudrate=9600, timeout=1)

time.sleep(2)  # wait for Arduino to start up
ser.flushInput()
ser.write(b'\xCC')
ser.write(b'\xFF')
ser.write(b'\xC0')
time.sleep(1)

while ser.in_waiting > 0:
    msg = ser.read(1)
    print("Message from Arduino:")
    print(msg)
