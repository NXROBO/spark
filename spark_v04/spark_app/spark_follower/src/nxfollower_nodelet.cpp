
#ifndef SPARK_APP_SPARK_FOLLOWER_SRC_NXFOLLOWER_NODELET_HPP_
#define SPARK_APP_SPARK_FOLLOWER_SRC_NXFOLLOWER_NODELET_HPP_

#include <nodelet/nodelet.h>

#include "nxfollower.hpp"
// this should really be in the implementation (.cpp file)

namespace nxfollower
{

class NxFollowerNodelet: public nodelet::Nodelet
{
protected:
	NxFollowerNode *nxfollower_node;
	ros::NodeHandle nh,pnh;
	virtual void onInit()
	{
		ROS_INFO("Start to bring up nxfollower!");
		nh 	= getNodeHandle();
		pnh = getPrivateNodeHandle();
		nxfollower_node = new NxFollowerNode(nh,pnh);
	}

public:
	~NxFollowerNodelet()
	{
		if (nxfollower_node != NULL)
		{
			delete nxfollower_node;
			nxfollower_node = NULL;
		}
	}
};

}

// watch the capitalization carefully
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(nxfollower::NxFollowerNodelet, nodelet::Nodelet)
//PLUGINLIB_DECLARE_CLASS(usb_cam, usb_cam_nodelet, usb_cam::usb_cam_nodelet, nodelet::Nodelet);

#endif
/*
 int main( int argc, char** argv )
 {
 ros::init(argc,argv,"orbcamera_node");
 ros::NodeHandle nh;
 NxCamera orb_cam(nh);
 orb_cam.Init();
 ros::AsyncSpinner spinner(6); // Use 4 threads
 spinner.start();
 ros::waitForShutdown();
 return 0;
 }
 */

