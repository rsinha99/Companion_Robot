import numpy as np
import serial
import time

isWindowsOS = True

if isWindowsOS:
    ser = serial.Serial(port='COM5', baudrate=9600, timeout=1)
else:
    ser = serial.Serial(port="/dev/ttyUSB0", baudrate=9600, timeout=1)

time.sleep(2)  # wait for Arduino to start up
ser.flushInput()
value = 0
thing = str(6)
ser.write(b'\xCC')
time.sleep(1)
ser.write(b'\xFF')
time.sleep(1)
ser.write(b'\xC0')
time.sleep(1)

while ser.in_waiting > 0:
    msg = ser.read(ser.inWaiting())
    print("Message from Arduino:")
    print(msg)
