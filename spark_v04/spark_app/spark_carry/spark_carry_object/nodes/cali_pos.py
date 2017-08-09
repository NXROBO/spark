#!/usr/bin/env python
import sys
import rospy
import time
from std_msgs.msg import String
from spark_carry_object.msg import *

def talker():
	global sta
	pub1 = rospy.Publisher('position_write_topic', position, queue_size = 1)
	pub2 = rospy.Publisher('cali_pix_topic', status, queue_size = 1)
	pos = position()
	sta = status()
	rospy.init_node('cali_pos', anonymous=True)

	r1 = rospy.Rate(1)  # 1s
	r2 = rospy.Rate(0.08) # 13s
	r3 = rospy.Rate(0.2)  # 5s

	r1.sleep()
	pos.x = 120
	pos.y = 0
	pos.z = 35
	pub1.publish(pos)
	r2.sleep()
		
	for i in range(20):	
		pos.x = 150 + i * 5
		pos.y = 200 - i * 10
		pos.z = -120
		pub1.publish(pos)
		r2.sleep()
		pub2.publish(1)


if __name__ == '__main__': 
	try: 
		talker() 
	except rospy.ROSInterruptException: 
		pass
