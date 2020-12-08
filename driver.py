import numpy as np
import serial
import time
from threading import Thread, Event
from binascii import hexlify



# TODO function to convert string to Serial Code for better user experience

isWindowsOS = False

if isWindowsOS:
    port = 'COM5'
else:
    port = '/dev/ttyUSB0'

ser = serial.Serial(port=port, baudrate=9600, timeout=1)

time.sleep(2)  # wait for Arduino to start up
ser.flushInput()

shared_arr = [0]
event = Event()


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
        print(f"shared data: {shared_arr}")
        time.sleep(1)


def forward():
    ser.write(b'\x00')
    ser.write(b'\x02')
    ser.write(b'\x00')

def right():
    ser.write(b'\x40')
    ser.write(b'\x02')
    ser.write(b'\x40')

def left():
    ser.write(b'\x80')
    ser.write(b'\x02')
    ser.write(b'\x80')


def nav_test():
    ser.write(b'\x02')  # Enter Following Mode
    while True:
        ser.write(b'\x40')
        time.sleep(5)
        ser.write(b'\x80')
        time.sleep(5)
        ser.write(b'\xC0')
        time.sleep(5)

    ser.write(b'\x03')  # Enter Calibration Mode
    time.sleep(2)
    
    while True:
        if ser.in_waiting > 0:
            msg = ser.read(1)
            print("Message from Arduino:")
            print(hexlify(msg))


if __name__ == "__main__":
    print("starting...")
    nav_test()
    # COMMENTED OUT BELOW: USED FOR REFERENCE WHEN STARTING THREADS
    # read = Thread(target=thread_read)
    # write = Thread(target=thread_modify)
    # read.start()
    # write.start()
    # try:
    #     while True:
    #         time.sleep(1)
    # except KeyboardInterrupt:
    #     event.set()
    #     read.join()
    #     write.join()
    #     print("Keyboard Interrupt")



