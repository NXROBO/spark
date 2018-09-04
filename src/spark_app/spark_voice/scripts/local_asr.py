#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import logging
import time
from threading import Thread, Event
from lib import Microphone

def main():
    logging.basicConfig(level=logging.INFO)
    quit_event = Event()
    mic = Microphone(quit_event=quit_event)
    pub = rospy.Publisher('voice/stt', String, queue_size=10)
    rospy.init_node('local_asr', anonymous=True)
    #rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
#        if mic.wakeup('alexa'):
#           print('Wake up\n')
        data = mic.listen()
        text = mic.recognize(data)
        hello_str = "REC: %s" % text
        rospy.loginfo(hello_str)
        if text:
            pub.publish(text)
        #rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
