#include<ros/ros.h>
int main(int argc,char **argv){
    ros::init(argc,argv,"hello");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("HELLO,ROS!");
}
