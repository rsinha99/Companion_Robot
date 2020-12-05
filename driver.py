import numpy as np
import serial
import time
from binascii import hexlify



# TODO function to convert string to Serial Code for better user experience

def main():
    isWindowsOS = False

    if isWindowsOS:
        port = 'COM5'
    else:
        port = '/dev/ttyUSB0'

    print("Hello")
    ser = serial.Serial(port=port, baudrate=9600, timeout=1)

    time.sleep(2)  # wait for Arduino to start up
    ser.flushInput()

def forward():
    main()
    global following_mode
    if following_mode = True:
        ser.write(b'\x0')
    if following_mode = False:
        ser.write(b'\x02')
        following_mode = True
        ser.write(b'\x0')

def right():
    main()
    global following_mode
    if following_mode = True:
        ser.write(b'\x40')
    if following_mode = False:
        ser.write(b'\x02')
        following_mode = True
        ser.write(b'\x40')

def left():
    main()
    global following_mode
    if following_mode = True:
        ser.write(b'\x80')
    if following_mode = False:
        ser.write(b'\x02')
        following_mode = True
        ser.write(b'\x80')

def dance():
    main()
    global following_mode
    if following_mode = True:
        while True:
            ser.write(b'\x80')
            time.sleep(5)
            ser.write(b'\x40')
            time.sleep(5)

    if following_mode = False:
        ser.write(b'\x02')
        following_mode = True
        while True:
            ser.write(b'\x80')
            time.sleep(5)
            ser.write(b'\x40')
            time.sleep(5)

    #ser.write(b'\x02')  # Enter Following Mode
    #print("Enter following mode")
    #while True:
     #   ser.write(b'\x40')
     #   print("Left")
     #   time.sleep(5)
      #  ser.write(b'\x80')
      #  print("Right")
      #  time.sleep(5)

    # ser.write(b'\x03')  # Enter Calibration Mode
    # time.sleep(2)
    
    # while True:
    #     if ser.in_waiting > 0:
    #         msg = ser.read(1)
    #         print("Message from Arduino:")
    #         print(hexlify(msg))


if __name__ == "__main__":
    print("starting...")
    main()
