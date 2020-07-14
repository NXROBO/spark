#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
from std_msgs.msg import String
import logging
import time
from threading import Thread, Event
from lib import Microphone,AipSpeech

""" 你的 APPID AK SK """
APP_ID = '10475584'
API_KEY = 'm2zeD2S8ft8z6Y0oUblsIZhy'
SECRET_KEY = 'Y226Y4YCRzpYXy3E3MTG0xxyqRGZYG50'

def main():
    logging.basicConfig(level=logging.INFO)
    quit_event = Event()
    mic = Microphone(quit_event=quit_event)
    pub = rospy.Publisher('voice/stt', String, queue_size=10)
    client = AipSpeech(APP_ID, API_KEY, SECRET_KEY)
    rospy.init_node('ali_asr', anonymous=True)
    while not rospy.is_shutdown():
        data = mic.listen()
        result = client.asr(''.join(list(data)), 'pcm', 16000, {'dev_pid': 1537,})
        if result['err_no']==0 and len(result["result"][0])>1:
            text = result["result"][0]
            pub.publish(text)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
