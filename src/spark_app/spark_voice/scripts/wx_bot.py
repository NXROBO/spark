#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import chardet
from std_msgs.msg import String
from wxpy import *


def main():
    pub = rospy.Publisher('voice/stt', String, queue_size=10)
    rospy.init_node('wx_bot', anonymous=True)
    bot = Bot()
    myself=bot.self
    myself.send('Waiting for ...')
    @bot.register(User,TEXT,False)
    def print_messages(msg):
        if (msg.receiver==msg.sender):
            txt=msg.text
            tx=txt.encode('utf-8')
            #print type(tx)
            print(tx)
            pub.publish(tx)


if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
