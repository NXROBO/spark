#!/usr/bin/env python
import sys
import rospy
import time
from std_msgs.msg import String
from swiftpro.msg import *

def talker():
	global sta
	pub1 = rospy.Publisher('position_write_topic', position, queue_size = 1)
	pub2 = rospy.Publisher('cali_pix_topic', status, queue_size = 1)
	pos = position()
	sta = status()
	rospy.init_node('cali_pos', anonymous=True)
	
	try:
		time.sleep(1)
		pos.x = 120
		pos.y = 0
		pos.z = 35
		pub1.publish(pos)
		time.sleep(13)
		
		pos.x = 180
		pos.y = 210
		pos.z = -110
		pub1.publish(pos)
		time.sleep(13)
		pub2.publish(1)
		
		pos.x = 170
		pos.y = 180
		pos.z = -110
		pub1.publish(pos)
		time.sleep(5)
		pub2.publish(1)
	
		pos.x = 205
		pos.y = 140
		pos.z = -110
		pub1.publish(pos)
		time.sleep(5)
		pub2.publish(1)
	
		pos.x = 206
		pos.y = 185
		pos.z = -110
		pub1.publish(pos)
		time.sleep(5)
		pub2.publish(1)
		
		pos.x = 235
		pos.y = 80
		pos.z = -110
		pub1.publish(pos)
		time.sleep(5)
		pub2.publish(1)
	
		pos.x = 242
		pos.y = 105
		pos.z = -110
		pub1.publish(pos)
		time.sleep(5)
		pub2.publish(1)
		
		pos.x = 255
		pos.y = 35
		pos.z = -110
		pub1.publish(pos)
		time.sleep(5)
		pub2.publish(1)

		pos.x = 250
		pos.y = 0
		pos.z = -110
		pub1.publish(pos)
		time.sleep(5)
		pub2.publish(1)
	
		pos.x = 240
		pos.y = -50
		pos.z = -110
		pub1.publish(pos)
		time.sleep(5)
		pub2.publish(1)
	
	except:
		return


if __name__ == '__main__': 
	try: 
		talker() 
	except rospy.ROSInterruptException: 
		pass

