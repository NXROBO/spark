#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import subprocess


class VoiceTTS:
    def __init__(self):
        rospy.init_node('ekho_tts')
        
        # Subscribe to the /voice/tts topic.
        rospy.Subscriber('/voice/tts', String, self.tts_callback)        
        rospy.loginfo("Ready to receive TTS")       
                    

        
    def tts_callback(self, msg):        
        # Log the tts to the screen
        rospy.loginfo("TTS: " + str(msg.data))
	subprocess.call(["ekho",msg.data])
        


if __name__=="__main__":
    try:
        VoiceTTS()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("TTS terminated.")
