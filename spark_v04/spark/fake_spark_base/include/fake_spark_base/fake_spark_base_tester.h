#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace nxspark
{
#define R 0.266
class SparkStateSimulation
{
private:
  ros::NodeHandle nh;
  ros::Subscriber cmd_vel_sub;
  tf::TransformBroadcaster tf_broadcaster;

  double linear_x, linear_y, linear_z;
  double angular_x, angular_y, angular_z;

  double Odom_x;
  double Odom_y;
  double Odom_theta;

  double prev_t;
  double curr_t;

  std::string odom, odom2;
  std::string base_footprint, base_footprint2;

  double t;

  ros::Publisher odom_pub;

  // ros::Publisher fback_cmd_vel_pub;
  // ros::Publisher odom_reset_pub;

public:
  // constructor
  SparkStateSimulation();

  // destructor
  ~SparkStateSimulation();

  // define init function
  bool onInit(ros::NodeHandle nh);  // define car.drive function

  // define top function
  int runEverything(void);

  // define odom calculator
  void pubOdom(void);

  // define cmd_vel callback
  void commandVelocityCallback(const geometry_msgs::Twist::ConstPtr &cmd_vel);
};
}
