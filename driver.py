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

time.sleep(10)  # wait for Arduino to start up
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
    ser.write(b'\x02')
    time.sleep(1)

    ser.write(b'\xC0')
    time.sleep(5)
    ser.write(b'\x10')
    time.sleep(1)

def right():
    ser.write(b'\x02')
    time.sleep(1)
    ser.write(b'\x40')
    time.sleep(3)
    ser.write(b'\x10')
    time.sleep(3)
    ser.write(b'\x40')
    time.sleep(3)

def left():
    #ser.write(b'\x80')
    #time.sleep(1)
    ser.write(b'\x02')
    time.sleep(1)
    while True:
        ser.write(b'\x80')

def stop():
    ser.write(b'\x10')
    time.sleep(1)

def dance():
    ser.write(b'\x02')  # Enter Following Mode
    
    while True:
        ser.write(b'\x40')
        time.sleep(1)
        ser.write(b'\x80')
        time.sleep(1)
        if event.is_set():
            break

#        ser.write(b'\xC0')
 #       time.sleep(5)

 #   ser.write(b'\x03')  # Enter Calibration Mode
 #   time.sleep(2)
    
 #   while True:
 #       if ser.in_waiting > 0:
 #           msg = ser.read(1)
 #           print("Message from Arduino:")
 #           print(hexlify(msg))


def calibrate():
    ser.write(b'\x03')
    while True:
        if ser.in_waiting > 0:
            msg = ser.read(1)
            print("Message from Arduino")
            print(hexlify(msg))
            break



# TODO Modify the code to account for target being on the outer edges of the camera
# TODO Modify the camera thread code to have variables that store the width of the camera
def follow_thread():
    print("entered function")
    video_width = 720
    margin = 30
    left_threshold = video_width / 3 + margin
    right_threshold = video_width / 3 * 2 - margin
    event.clear()
    print("follow mode")
    ser.write(b'\x02')          # Set robot to Following Mode
    while True:
        if ser.in_waiting > 0:  # At the moment, Arduino does not send message to Nvidia
            if ser.read(1) == b'\x7f':
                print("Robot arrived")
                break
        if event.is_set():
            print("event is set")
            break

        if camera.currX == -1:    # No target to follow is detected
            print('turning left')
            ser.write(b'\x46')      # Turn left until target is found
            while camera.currX == -1 or camera.currX < left_threshold or camera.currX > right_threshold:
                print(camera.currX)
                time.sleep(1)
                # keep turning left until target is centered
        elif camera.currX > right_threshold:    # Color is to the right of robot
            print('forward right')
            ser.write(b'\xEE')                  # Drive Forward-right
        elif camera.currX < left_threshold:     # Target is to the left of robot
            print('foward left')
            ser.write(b'\xCE')                  # Drive Forward-left
        else:
            print('forward')
            ser.write(b'\xC0')                  # Drive straight

        print(camera.currX)
        time.sleep(1)

        # TODO if robot returns serial code "Obstacle", close this thread (break from loop)
#        print(camera.currX)
#        time.sleep(1)




if __name__ == "__main__":
    print("starting...")
    left()
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


