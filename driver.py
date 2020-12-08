import numpy as np
import serial
import time
from threading import Thread, Event
from binascii import hexlify
import camera



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


def calibrate():
    ser.write(b'\x03')
    time.sleep(2)


# TODO Modify the code to account for target being on the outer edges of the camera
def follow_thread():
    ser.write(b'\x02')          # Set robot to Following Mode
    while True:
        if event.is_set():
            break

        if camera.currX is None:    # No target to follow is detected
            ser.write(b'\x46')      # Turn left until target is found
            while camera.currX is None or camera.currX > 950 or camera.currX < 650:
                None    # keep turning left until target is centered
        elif camera.currX > 950:  # Color is to the right of robot
            ser.write(b'\xE2')      # Drive Forward-right
        elif camera.currX < 650:    # Target is to the left of robot
            ser.write(b'\xC2')      # Drive Forward-left
        else:
            ser.write(b'\xC0')

        # TODO if robot returns serial code "Obstacle", close this thread (break from loop)
        print(camera.currX)
        time.sleep(1)




if __name__ == "__main__":
    print("starting...")
    # color = Thread(target=camera.camera_thread)
    # color.start()
    # while True:
    #     try:
    #         if camera.currX is not None:
    #             print(camera.currX)
    #         time.sleep(1)
    #     except (KeyboardInterrupt, Exception):
    #         camera.event.set()
    #         color.join()
    #         break

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



