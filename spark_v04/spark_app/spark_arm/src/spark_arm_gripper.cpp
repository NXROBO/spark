#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <swiftpro/position.h>
#include <swiftpro/status.h>

class SparkArm
{
public:
  SparkArm();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle ph_, nh_;

  int x_direction_, y_direction_, z_direction_, gripper_number_;
  double x_max_, y_max_, z_max_;
  ros::Publisher pos_pub_;
  ros::Subscriber joy_sub_;
  ros::Publisher gripper_pub_;

  bool deadman_pressed_;

};

SparkArm::SparkArm():
  ph_("~"),
  x_direction_(1),
  y_direction_(2),
  z_direction_(3),
  gripper_number_(0),
  x_max_(300),
  y_max_(3),
  z_max_(50)
{
  deadman_pressed_ = false;
  pos_pub_ = ph_.advertise<swiftpro::position>("/position_write_topic", 1, true);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 10, &SparkArm::joyCallback, this);
  gripper_pub_ = ph_.advertise<swiftpro::status>("/gripper_topic", 1, true);
}


//callback function that translate joystick motion
void SparkArm::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{ 
  swiftpro::position pos;
  pos.x = x_max_*joy->axes[x_direction_];
  pos.y = y_max_*joy->axes[y_direction_];
  pos.z = z_max_*joy->axes[z_direction_];
  pos_pub_.publish(pos);
  ros::Rate loop_rate(2);
  loop_rate.sleep();
  ROS_INFO("x coordinate: %f", pos.x);
  ROS_INFO("y coordinate: %f", pos.y);
  ROS_INFO("z coordinate: %f", pos.z);

  
  deadman_pressed_ = joy->buttons[gripper_number_];
  swiftpro::status sta;
  
  if (deadman_pressed_)
  {
    sta.status = 1;
  }
  else if(!deadman_pressed_ )
  {
    sta.status = 0;
  }  

  gripper_pub_.publish(sta);
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "spark_arm_joystick");
  SparkArm spark_arm_joystick;

  ros::spin();
}
