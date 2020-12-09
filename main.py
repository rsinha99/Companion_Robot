from flask import Flask
from flask_ask import Ask, statement
#import facerec_from_webcam_faster as facerec
import threading
import driver
import test_robot_chat_client as chat
import numpy as np
import serial
import time
from binascii import hexlify

app = Flask(__name__)
ask = Ask(app, '/')

#following_mode = False
#isWindowsOS = False

#if isWindowsOS:
#    port = 'COM5'
#else:
#    port = '/dev/ttyUSB0'
#ser = serial.Serial(port=port, baudrate=9600, timeout=1)


@ask.intent('WhoFace')

#def who_face():
 #   speech_text = 'Hi beautiful, woops I mean {}'.format(facerec.name)
 #   return statement(speech_text).simple_card('Johnny', speech_text)

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
    driver.follow()
    speech_text = 'Following'
    return statement(speech_text).simple_card('Johnny', speech_text)

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

@ask.intent('Party')
def party():
    chat()

@ask.intent('Camera')
def camera():
  camera.camera_thread()
  

if __name__== '__main__':
    #face_rec = threading.Thread(target=facerec.facerec_thread)
    #face_rec.start()
    app.run()
