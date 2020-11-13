import numpy as np
import serial

ser = serial.Serial(port="/dev/ttyUSB0", baudrate=9600, timeout=1)
ser.flushinput()
value = 0

while True:
    byte = value.to_bytes(value, byteorder='little')
    if byte == b'\xFF':
        break
    ser.write(byte)
    value += 1
