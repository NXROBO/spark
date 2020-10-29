#!/usr/bin/env python
import sys
import rospy
import time
import _thread
from std_msgs.msg import String
from spark_carry_object.msg import *
start_cali = 0
def msg_callback(data):
	global start_cali
	if(data.data == "start"):
		start_cali = 1
	else:
		start_cali = 0
def talker(threadName, delay):
	global sta
	global start_cali

	pub1 = rospy.Publisher('position_write_topic', position, queue_size = 5)
	pub2 = rospy.Publisher('cali_pix_topic', status, queue_size = 5)
	pos = position()
	sta = status()

	sub3 = rospy.Subscriber("start_topic", String, msg_callback)
	r1 = rospy.Rate(1)  # 1s
	r2 = rospy.Rate(0.05) # 20s
	r3 = rospy.Rate(0.2)  # 5s
	r4 = rospy.Rate(0.4)  # 2.5s

	r1.sleep()
	pos.x = 120
	pos.y = 0
	pos.z = 35
	pub1.publish(pos)
	r2.sleep()
	print ("wait to start----")
	while(start_cali==0):
		r1.sleep()
	if(start_cali == 0):
		return
	else :
		r1.sleep()
		r1.sleep()
		r1.sleep()
		r1.sleep()
		r1.sleep()
	print ("start to move the uarm----")
	for i in range(20):	
		if rospy.is_shutdown():
			break
		r1.sleep()
		pos.x = 180 + i * 5
		pos.y = 200 - i * 10
		pos.z = -110
		pub1.publish(pos)
		r1.sleep()
		if i == 0:
			r2.sleep()
			
		else:
			r4.sleep()
		print(pos.x, pos.y)
		pub2.publish(1)
		r4.sleep()
	pos.x = 120
	pos.y = 0
	pos.z = 35
	pub1.publish(pos)

	r3.sleep()

if __name__ == '__main__': 
	rospy.init_node('cali_pos', anonymous=True)
	sub3 = rospy.Subscriber("start_topic", String, msg_callback)
	_thread.start_new_thread(talker, ("Thread-1", 2, ) )
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")


