import numpy as np
import serial
import time
from binascii import hexlify

isWindowsOS = False

if isWindowsOS:
    port = 'COM5'
else:
    port = '/dev/ttyUSB0'

# TODO function to convert string to Serial Code for better user experience

def main():

    ser = serial.Serial(port=port, baudrate=9600, timeout=1)

    time.sleep(2)  # wait for Arduino to start up
    ser.flushInput()
    # ser.write(b'\x03')  # Enter Calibration Mode
    ser.write(b'\C3')
    time.sleep(2)

    while (True):
        if ser.in_waiting > 0:
            msg = ser.read(1)
            print("Message from Arduino:")
            print(hexlify(msg))


if __name__ == "__main__":
    print("starting...")
    main()
