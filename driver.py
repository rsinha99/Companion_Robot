import numpy as np
import serial
import time
from threading import Thread, Event
from binascii import hexlify

isWindowsOS = False

if isWindowsOS:
    port = 'COM5'
else:
    port = '/dev/ttyUSB0'

shared_arr = [0]
event = Event()

# TODO function to convert string to Serial Code for better user experience

def nav_test():

    ser = serial.Serial(port=port, baudrate=9600, timeout=1)

    time.sleep(2)  # wait for Arduino to start up
    ser.flushInput()

    ser.write(b'\x02')  # Enter Following Mode
    while True:
        ser.write(b'\x40')
        time.sleep(5)
        ser.write(b'\x80')
        time.sleep(5)

    # ser.write(b'\x03')  # Enter Calibration Mode
    # time.sleep(2)
    
    # while True:
    #     if ser.in_waiting > 0:
    #         msg = ser.read(1)
    #         print("Message from Arduino:")
    #         print(hexlify(msg))


def thread_modify():
    while True:
        if event.is_set():
            break
        shared_arr[0] += 1
        time.sleep(1)


def thread_read():
    while True:
        if event.is_set():
            break
        print(f"shared data: {shared_arr[0]}")
        time.sleep(1)


if __name__ == "__main__":
    print("starting...")
    write = Thread(target=thread_modify())
    read = Thread(target=thread_read())

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Keyboard Interrupt was CAUGHT")
        event.set()

    # nav_test()
