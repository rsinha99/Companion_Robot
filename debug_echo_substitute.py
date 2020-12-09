# This script is used in place of the Amazon Echo for quicker testing
import driver
# import camera
from threading import Thread
# import facerec_from_webcam_faster as face_rec
import time

threads = []
newCommand = False


def forward():
    forward_thread = Thread(target=driver.forward)
    forward_thread.start()
    threads.append(forward_thread)


def left():
    None


def right():
    None


def follow_me():
    follow_thread = Thread(target=driver.follow_thread)
    follow_thread.start()
    threads.append(follow_thread)


def dance():
    None


if __name__ == '__main__':
    # cam_thread = Thread(target=camera.camera_thread)
    # face_rec_thread = Thread(target=face_rec.facerec_thread)
    # face_rec_thread.start()
    # threads.append(face_rec_thread)
    # cam_thread.start()
    # threads.append(cam_thread)
    # forward()
    driver.forward()

    try:
        while True:
            None
            # command = input()
            # if command == 'follow':
            #     follow_me()
    except (KeyboardInterrupt, Exception):
        driver.event.set()
        # face_rec.event.set()
        # camera.event.set()
        for i in range(len(threads)):
            threads.pop().join()



