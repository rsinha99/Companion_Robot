from flask import Flask
from flask_ask import Ask, statement
import facerec_from_webcam_faster as facerec
import threading
import driver
import test_robot_chat_client as chat
import numpy as np
import serial
import time
import camera
from binascii import hexlify

app = Flask(__name__)
ask = Ask(app, '/')

threads = []
#following_mode = False
#isWindowsOS = False

#if isWindowsOS:
#    port = 'COM5'
#else:
#    port = '/dev/ttyUSB0'
#ser = serial.Serial(port=port, baudrate=9600, timeout=1)


@ask.intent('WhoFace')
def who_face():
    print(facerec.name)
    if facerec.name == "unknown":
        print("in unkown")
        speech_text = 'Where are you'
    else:
        print("in becca")
        dance()
        speech_text = 'Hi beautiful, woops I mean {}'.format(facerec.name)
         
    return statement(speech_text).simple_card('Johnny', speech_text)

@ask.intent('Forward')
def forward():
    driver.forward()
    speech_text = 'Moving forward' 
    return statement(speech_text).simple_card('Johnny', speech_text)

@ask.intent('Left')
def left():
    driver.left()
    speech_text = 'Turning left'
    return statement(speech_text).simple_card('Johnny', speech_text)

@ask.intent('Right')
def right():
    driver.right()
    speech_text = 'Turning right'
    return statement(speech_text).simple_card('Johnny', speech_text)

@ask.intent('Follow')
def follow_me():
    follow_thread = threading.Thread(target=driver.follow_thread, name="following_thread")
    follow_thread.start()
    threads.append(follow_thread)

    speech_text = 'Following'
    return statement(speech_text).simple_card('Johnny', speech_text)

@ask.intent('StopFollow')
def stop_follow():
    driver.event.set()
    for i in range(len(threads)):
        if threads[i].name == "following_thread":
            threads.pop(i).join()

@ask.intent('Dance')
def dance():
    driver.event.clear()
    driver_thread= threading.Thread(target=driver.dance)
    driver_thread.start()
    speech_text = 'Party time'
    return statement(speech_text).simple_card('Johnny', speech_text)

@ask.intent('Stop')
def stop():
    driver.event.set()
    time.sleep(1)
    driver.stop()
    speech_text = 'Skrttt'
    return statement(speech_text).simple_card('Johnny', speech_text)

#@ask.intent('Party')
#def party():
#    chat()


  

if __name__== '__main__':
    face_rec = threading.Thread(target=facerec.facerec_thread)
    face_rec.start()
    

    cam_thread = threading.Thread(target=camera.camera_thread)
    # face_rec_thread = Thread(target=face_rec.facerec_thread)
    # face_rec_thread.start()
    # threads.append(face_rec_thread)
    cam_thread.start()
    threads.append(cam_thread)
    # forward()
    # driver.forward()
    app.run()
