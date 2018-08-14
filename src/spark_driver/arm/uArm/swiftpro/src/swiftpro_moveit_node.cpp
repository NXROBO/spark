/* 
 * Software License Agreement (BSD License)
 * Copyright (c) 2017, UFactory, Inc.
 * All rights reserved.
 * Author: Roger Cui  <roger@ufactory.cc>   
 */
#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <swiftpro/position.h>

#define MATH_PI 				3.141592653589793238463
#define MATH_TRANS  			57.2958    
#define MATH_L1 				106.6
#define MATH_L2 				13.2
#define MATH_LOWER_ARM 			142.07
#define MATH_UPPER_ARM 			158.81

float motor_angle[3] = {90.0, 90.0, 0.0};
	
/* 
 * Description: forward kinematics of swift pro
 * Inputs: 		angle[3]			3 motor angles(degree)
 * Outputs:		position[3]			3 cartesian coordinates: x, y, z(mm)
 */
void swift_fk(float angle[3], float position[3])
{
	double stretch = MATH_LOWER_ARM * cos(angle[1] / MATH_TRANS) 
				   + MATH_UPPER_ARM * cos(angle[2] / MATH_TRANS) + MATH_L2 + 56.55;

	double height = MATH_LOWER_ARM * sin(angle[1] / MATH_TRANS) 
				  - MATH_UPPER_ARM * sin(angle[2] / MATH_TRANS) + MATH_L1;
	
	position[0] = stretch * sin(angle[0] / MATH_TRANS);
	position[1] = -stretch * cos(angle[0] / MATH_TRANS);
	position[2] = height - 74.55;
}


/* 
 * Description: callback when receive data from move_group/fake_controller_joint_states
 * Inputs: 		msg					3 necessary joints for kinematic chain(degree)
 * Outputs:		motor_angle[3]		3 motor angles(degree)
 */
void joint_Callback(const sensor_msgs::JointState& msg)
{
	motor_angle[0] = msg.position[0] * 57.2958 + 90;
	motor_angle[1] = 90 - msg.position[1] * 57.2958;
	motor_angle[2] = (msg.position[1] + msg.position[2]) * 57.2958;
}


/* 
 * Node name:
 *	 swiftpro_moveit_node
 *
 * Topic publish: (rate = 20Hz, queue size = 1)
 *   position_write_topic
 *
 * Topic subscribe: (queue size = 1)
 *   move_group/fake_controller_joint_states
 */
int main(int argc, char **argv)
{
	float position[3];
	swiftpro::position pos;
	
	ros::init(argc, argv, "swiftpro_moveit_node");
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("move_group/fake_controller_joint_states", 1, joint_Callback);
	ros::Publisher 	pub = n.advertise<swiftpro::position>("position_write_topic", 1);
	ros::Rate loop_rate(20);
	
	while (ros::ok())
	{
		swift_fk(motor_angle, position);
		pos.x = position[0];
		pos.y = position[1];
		pos.z = position[2];
		pub.publish(pos);
		
		ros::spinOnce();
		loop_rate.sleep();
	}		
	return 0;
}

