#include "ros/ros.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <boost/function.hpp>
#include <boost/thread/thread.hpp>
#include "spark_base/GyroMessage.h"
#include "spark_base/SparkBaseSensor.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class TopicSubscribeTest
{
public:
  ros::NodeHandle nhandle;
  bool point_clound_state, depth_image_state, rgb_image_state, gyro_state, odom_state, ir_bumper_cliff_state;
  boost::thread *test_topic_thread_;
  TopicSubscribeTest(ros::NodeHandle in_nh)
  {
    point_clound_state = false;
    depth_image_state = false;
    rgb_image_state = false;
    gyro_state = false;
    odom_state = false;
    ir_bumper_cliff_state = false;
    nhandle = in_nh;

    test_topic_thread_ = new boost::thread(boost::bind(&TopicSubscribeTest::testTopicStatus, this));
  }

  ~TopicSubscribeTest()
  {
    if (test_topic_thread_ != NULL)
    {
      delete test_topic_thread_;
    }
  }

  bool testTopicStatus()
  {
    // test point cloud topic
    if (testPointCloudTopic())
      ROS_INFO("point cloud status:[TRUE]");
    else
      ROS_ERROR("point cloud status:[FALSE]");
    // test
    if (testDepthImageTopic())
      ROS_INFO("depth image status:[TRUE]");
    else
      ROS_ERROR("depth image status:[FALSE]");

    if (testRgbImageTopic())
      ROS_INFO("rgb image status:[TRUE]");
    else
      ROS_ERROR("rgb image status:[FALSE]");

    if (testGyroTopic())
      ROS_INFO("gyro status:[TRUE]");
    else
      ROS_ERROR("gyro status:[FALSE]");

    if (testOdomTopic())
      ROS_INFO("odom status:[TRUE]");
    else
      ROS_ERROR("odom status:[FALSE]");

    if (testIrBumperCliffTopic())
      ROS_INFO("ir bumper cliff status:[TRUE]");
    else
      ROS_ERROR("ir bumper cliff status:[FALSE]");
    return true;
  }
  // test point cloud image topic
  bool testPointCloudTopic()
  {
    ros::Subscriber pcloud_sub =
        nhandle.subscribe<PointCloud>("/camera/depth/points", 1, &TopicSubscribeTest::pointCloudCb, this);
    point_clound_state = false;
    ros::Rate rate(10);
    int max_wait_time = 5;
    double prev_time = ros::Time().now().toSec();
    while (1)
    {
      if (ros::Time().now().toSec() - prev_time > max_wait_time)
      {
        pcloud_sub.shutdown();
        return false;
      }
      if (point_clound_state)
      {
        pcloud_sub.shutdown();
        return true;
      }
      rate.sleep();
    }
  }

  void pointCloudCb(const PointCloud::ConstPtr &cloud)
  {
    point_clound_state = true;
  }

  // test depth image topic
  bool testDepthImageTopic()
  {
    ros::Subscriber sub =
        nhandle.subscribe<sensor_msgs::Image>("/camera/depth/image", 1, &TopicSubscribeTest::depthImageCb, this);
    depth_image_state = false;
    ros::Rate rate(10);
    int max_wait_time = 5;
    double prev_time = ros::Time().now().toSec();
    while (1)
    {
      if (ros::Time().now().toSec() - prev_time > max_wait_time)
      {
        sub.shutdown();
        return false;
      }
      if (depth_image_state)
      {
        sub.shutdown();
        return true;
      }
      rate.sleep();
    }
  }
  void depthImageCb(const sensor_msgs::Image::ConstPtr &image_msg)
  {
    depth_image_state = true;
  }

  // test rgb image topic
  bool testRgbImageTopic()
  {
    ros::Subscriber sub =
        nhandle.subscribe<sensor_msgs::Image>("/camera/rgb/image_raw", 1, &TopicSubscribeTest::rgbImageCb, this);
    rgb_image_state = false;
    ros::Rate rate(10);
    int max_wait_time = 5;
    double prev_time = ros::Time().now().toSec();
    while (1)
    {
      if (ros::Time().now().toSec() - prev_time > max_wait_time)
      {
        sub.shutdown();
        return false;
      }
      if (rgb_image_state)
      {
        sub.shutdown();
        return true;
      }
      rate.sleep();
    }
  }
  void rgbImageCb(const sensor_msgs::Image::ConstPtr &image_msg)
  {
    rgb_image_state = true;
  }

  // test gyro topic
  bool testGyroTopic()
  {
    ros::Subscriber sub =
        nhandle.subscribe<spark_base::GyroMessage>("/spark_base/gyro", 1, &TopicSubscribeTest::gyroCb, this);
    gyro_state = false;
    ros::Rate rate(10);
    int max_wait_time = 5;
    double prev_time = ros::Time().now().toSec();
    while (1)
    {
      if (ros::Time().now().toSec() - prev_time > max_wait_time)
      {
        sub.shutdown();
        return false;
      }
      if (gyro_state)
      {
        sub.shutdown();
        return true;
      }
      rate.sleep();
    }
  }
  void gyroCb(const spark_base::GyroMessage::ConstPtr &gyro_msg)
  {
    gyro_state = true;
  }

  // test odom topic
  bool testOdomTopic()
  {
    ros::Subscriber sub = nhandle.subscribe<nav_msgs::Odometry>("/odom", 1, &TopicSubscribeTest::odomCb, this);
    odom_state = false;
    ros::Rate rate(10);
    int max_wait_time = 5;
    double prev_time = ros::Time().now().toSec();
    while (1)
    {
      if (ros::Time().now().toSec() - prev_time > max_wait_time)
      {
        sub.shutdown();
        return false;
      }
      if (odom_state)
      {
        sub.shutdown();
        return true;
      }
      rate.sleep();
    }
  }
  void odomCb(const nav_msgs::Odometry::ConstPtr &odom_msg)
  {
    odom_state = true;
  }

  // test ir bumper cliff topic
  bool testIrBumperCliffTopic()
  {
    ros::Subscriber sub = nhandle.subscribe<spark_base::SparkBaseSensor>("/spark_base/sensor", 1,
                                                                         &TopicSubscribeTest::irBumperCliffCb, this);
    ir_bumper_cliff_state = false;
    ros::Rate rate(10);
    int max_wait_time = 5;
    double prev_time = ros::Time().now().toSec();
    while (1)
    {
      if (ros::Time().now().toSec() - prev_time > max_wait_time)
      {
        sub.shutdown();
        return false;
      }
      if (ir_bumper_cliff_state)
      {
        sub.shutdown();
        return true;
      }
      rate.sleep();
    }
  }
  void irBumperCliffCb(const spark_base::SparkBaseSensor::ConstPtr &sensor_msg)
  {
    ir_bumper_cliff_state = true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "spark_test_node");
  ROS_INFO("Spark test bring up");

  ros::NodeHandle n;
  TopicSubscribeTest test(n);

  // start to test the spark
  // test.testTopicStatus();
  ROS_INFO("Spark test stop");
  ros::spin();

  return 0;
}
