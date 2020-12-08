# This script is used in place of the Amazon Echo for quicker testing
import driver
import camera
from threading import Thread

threads = []
newCommand = False


def forward():
    None


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
    cam_thread = Thread(target=camera.camera_thread)
    cam_thread.start()
    threads.append(cam_thread)

    try:
        while True:
            command = input()
            if command == 'follow':
                follow_me()
    except (KeyboardInterrupt, Exception):
        driver.event.set()
        camera.event.set()
        for i in range(len(threads)):
            threads.pop().join()



