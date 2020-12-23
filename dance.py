from robot_chat_client import RobotChatClient
from flask_ask import Ask, statement
import time
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
def test_callback(message_dict):
    print('Received dictionary {}'.format(message_dict))
    print('The message is type {}'.format(message_dict['type']))

    if message_dict['type'] == 'test_message_type':
        print('Value of field foo: {}'.format(message_dict['foo']))

    if message_dict['type'] == 'users':
        print('Number of users: {}'.format(message_dict['count']))

    if message_dict['type'] == 'dance':
      #print('Party Time')
      driver.event.clear()
      driver.dance()
      #driver_thread= threading.Thread(target=driver.dance)
      #driver_thread.start()
      #speech_text = 'Party time'
      #return statement(speech_text).simple_card('Johnny', speech_text)

    
# Run this script directly to invoke this test sequence
if __name__ == '__main__':
    print('Creating RobotChatClient object')
    client = RobotChatClient('ws://3.80.184.4:5001', callback=test_callback)

    time.sleep(1)
    print('Sending a dance message')
    client.send({'type':'dance'})

    print('Waiting for ctrl+c')
