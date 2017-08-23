#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/thread.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <pthread.h>
#include <sys/types.h>
#include <stdbool.h>
#include <signal.h>

#include <iostream>
#include <iostream>

#include "rslidar_driver.h"
#include "rslidar_protocol.h"
#include "rstypes.h"
#include "util.h"

using namespace rs::standalone::rslidar;
using namespace std;

namespace nxrobo
{
class Threeiladar
{
private:
  boost::thread *lada_thread_;
  ros::NodeHandle nh;

  ros::Publisher scan_pub;

  // modified by Xilai
  std::string opt_com_path;

  _u32 opt_com_baudrate;
  RSlidarDriver *drv;
  u_result op_result;
  _u8 eflag;

public:
  Threeiladar(ros::NodeHandle in_nh)
  {
    nh = in_nh;
    opt_com_path = "/dev/ttyUSB0";
    if (!nh.getParam("/b_3ilidar_node/lidar_USB", opt_com_path))
      ROS_ERROR("No lidar_USB param found, the serial port will be set as %s", opt_com_path.c_str());
    opt_com_baudrate = 115200;
    drv = NULL;

    scan_pub = nh.advertise<sensor_msgs::LaserScan>("/scan", 5);

    lada_thread_ = new boost::thread(boost::bind(&Threeiladar::laserThread, this));
  }

  ~Threeiladar()
  {
    lada_thread_->join();
    lada_thread_ = NULL;
  }

private:
  void laserThread()
  {
    ros::Rate rate(100);

    // create the driver instance
    drv = RSlidarDriver::CreateDriver(RSlidarDriver::DRIVER_TYPE_SERIALPORT);
    if (!drv)
    {
      printf("insufficent memory ,exit \n");
      exit(-2);
    }

    if (IS_FAIL(drv->connect(opt_com_path.c_str(), opt_com_baudrate)))
    {
      printf(" Error ,cannot bind to the specified serial port \n");
      return;
    }

    // start scan
    if (IS_FAIL(drv->startScan()))
    {
      return;
    }

    while (ros::ok())
    {
      rate.sleep();
      // public laser
      ROS_INFO("To publish laser");
      publicLaser();
    }

    RSlidarDriver::DisposeDriver(drv);
  }

  void publicLaser()
  {
    RSLIDAR_SIGNAL_DISTANCE_UNIT_T nodes[360 * 2];
    size_t count = _countof(nodes);
    double t1 = ros::Time().now().toSec();
    op_result = drv->grabScanData(nodes, count);
    double t2 = ros::Time().now().toSec();
    ROS_INFO("cost time:%f", t2 - t1);
    if (count <= 50)
    {
      eflag = 0;
      drv->getErrorInfo(&eflag);
      printf(" >>>>>>>>>>>>>>>>>>>>>>>>>>>>>002 eflag = %x \n", eflag);
      return;
    }

    // publish laser
    int num_readings = 360 * 2;
    int laser_frequency = 5;
    sensor_msgs::LaserScan scan;
    scan.header.stamp = ros::Time().now();
    scan.header.frame_id = "/lidar_link";
    scan.angle_min = 0;
    scan.angle_max = 6.28;
    scan.angle_increment = 6.28 / num_readings;
    scan.time_increment = (1.0 / laser_frequency) / (num_readings);
    scan.range_min = 0.2;
    scan.range_max = 8;

    scan.ranges.resize(num_readings);
    scan.ranges.assign(num_readings, std::numeric_limits<double>::infinity());
    // scan.intensities.resize(num_readings);
    double ascale = 6.28 / (100.0 * 360);
    float dist = 0;
    for (unsigned int i = 0; i < count; i++)
    {
      int index = (nodes[i].angle * ascale - scan.angle_min) / scan.angle_increment;
      // cout<<index<<endl;
      dist = nodes[i].distanceValue * 0.25 / 1000.0;
      if (dist >= scan.range_min && dist <= scan.range_max)
        scan.ranges[index] = dist;

      // scan.intensities[index] = nodes[i].distanceValue*0.25/1000.0;
    }
    scan_pub.publish(scan);
    // cout<<(int)count<<endl;
    /*for (int i = 0; i < count; i++) {
      cout << (int) nodes[i].angle << "," << (int) nodes[i].distanceValue
          << ";";
    }*/
  }
};
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "b_3ilidar_node");
  ros::NodeHandle n;

  nxrobo::Threeiladar tladar(n);
  ros::spin();
  return 0;
}
