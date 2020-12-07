from flask import Flask
from flask_ask import Ask, statement
import facerec_from_webcam_faster as facerec
import threading
import driver
import test_robot_chat_client as chat

app = Flask(__name__)
ask = Ask(app, '/')

if isWindowsOS:
    port = 'COM5'
else:
    port = '/dev/ttyUSB0'
ser = serial.Serial(port=port, baudrate=9600, timeout=1)


@ask.intent('WhoFace')

def who_face():
    speech_text = 'Hi beautiful, woops I mean {}'.format(facerec.name)
    return statement(speech_text).simple_card('Johnny', speech_text)

def forward():
    driver.forward()
    speech_text = 'Moving forward' 
    return statement(speech_text).simple_card('Johnny', speech_text)

def left():
    driver.left()
    speech_text = 'Turning left'
    return statement(speech_text).simple_card('Johnny', speech_text)

def right():
    driver.right()
    speech_text = 'Turning right'
    return statement(speech_text).simple_card('Johnny', speech_text)

def follow_me():
    driver.follow()
    speech_text = 'Following'
    return statement(speech_text).simple_card('Johnny', speech_text)

@ask.intent('Dance')
def dance():
    driver.dance()
    speech_text = 'Party time'
    return statement(speech_text).simple_card('Johnny', speech_text)

def party():
    chat()

__name__== '__main__':
    face_rec = threading.Thread(target=facerec.facerec_thread)
    face_rec.start()
    app.run()
