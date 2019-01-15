#include "ros/ros.h"
#include <cstdlib>
#include <iostream>
#include <string>
#include "std_msgs/String.h"

#include <boost/function.hpp>
#include <boost/thread/thread.hpp>
#include "spark_base/SparkBaseDock.h"
#include "spark_base/GyroMessage.h"
#include "spark_base/SparkBaseOdom.h"
#include <nav_msgs/Odometry.h>    // odom
#include <geometry_msgs/Twist.h>  // cmd_vel

using namespace std;

class Go2DockNode
{
private:

    ros::NodeHandle n;
    boost::thread *thrdc;
    boost::thread *thrdcbv;
    //ÏûÏ¢·¢²¼Æ÷
    ros::Publisher pub_go2dock;

    ros::Publisher pub_search4dock;

    ros::Publisher pub_vel;
    //¶¨ÔÄÏûÏ¢
    ros::Subscriber sub_dock;
    ros::Subscriber sub_gyro;

    int search_dock;
    float total_yaw;
    float last_yaw_value;
    int gotDockMsg;
public:
    /**
     * 	¹¹Ôìº¯Êý
     */
    Go2DockNode(ros::NodeHandle nb, int model)
    {
        n = nb;
        thrdc = NULL;
		thrdcbv = NULL;
        search_dock = 0;
        pub_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        pub_go2dock = n.advertise<std_msgs::String>("/spark_base/handle_go2dock", 10);
        pub_search4dock = n.advertise<std_msgs::String>("/spark_base/handle_search_dock", 1);
//        thrdcbv = new boost::thread(boost::bind(&Go2DockNode::dealGo2DockThread, this));
		if(model == 1)
			thrdcbv = new boost::thread(boost::bind(&Go2DockNode::dealGo2DockThread, this));
		else if(model == 0)
			thrdcbv = new boost::thread(boost::bind(&Go2DockNode::turnTestThread, this)); 
    }
    /**
     * 	ÎöÔìº¯Êý
     */
    ~Go2DockNode()
    {
        destroyThread(&thrdcbv);
        destroyThread(&thrdc);
    }


    /**
     * 	Ïû»ÙÏß³Ì
     */
    bool destroyThread(boost::thread **th)
    {

        if((*th) != NULL)
        {
            (*th)->interrupt();
            (*th)->join();
            delete (*th);
            (*th) = NULL;
            return true;
        }
        return true;
    }


	void pubGo2Dock(int flag)
    {
        //-----just for dock----
        std_msgs::String msg;
        if(flag == 0)
            msg.data = "close";
        else
            msg.data = "open";
        pub_go2dock.publish(msg);
        ROS_WARN("pub_go2dock %s",msg.data.c_str());
        //----end dock--------
    }

    void searchDock(int flag)
    {
        std_msgs::String msg;
        if(flag == 0)
            msg.data = "close";
        else
            msg.data = "open";
        pub_search4dock.publish(msg);
        ROS_WARN("pub_search4dock %s",msg.data.c_str());
    }
    void getGyroDataCallback(const spark_base::GyroMessage::ConstPtr& msg)
    {
        static float last_yaw = 0;
        float yaw = msg->yaw+180;
        if(total_yaw == 999)
        {
            last_yaw = yaw;
            total_yaw = 0;
        }
        float diff_value = ((yaw-last_yaw) >= 0) ? (yaw-last_yaw) : (((yaw-last_yaw)<-180) ? (360+yaw-last_yaw) : (yaw-last_yaw));
        total_yaw = total_yaw+diff_value;
        last_yaw = yaw;
    }
    void getDockStatusCallback(const spark_base::SparkBaseDock::ConstPtr& msg)
    {
        static int count = 0;
        if(msg->dock_dir_BACK || msg->dock_dir_front || msg->dock_dir_left || msg->dock_dir_right)
        {

            if(gotDockMsg == 0)
            {
                count = 0;
                gotDockMsg = 1;
            }
            if((msg->dock_dir_front==true)&&(msg->dock_dir_left==false) && (msg->dock_dir_right==false))
            {
                count++;
                if(count > 20)
                    gotDockMsg = 2;
            }

        }
    }
    int turnAndSearchDock(float speed)
    {
		geometry_msgs::Twist cmdvel_;
        total_yaw = 999;
        gotDockMsg = 0;
        sub_gyro = n.subscribe<spark_base::GyroMessage>("/spark_base/gyro", 1, &Go2DockNode::getGyroDataCallback, this);
        sub_dock = n.subscribe<spark_base::SparkBaseDock>("/spark_base/dock", 10, &Go2DockNode::getDockStatusCallback, this);
        while(ros::ok() && total_yaw == 999)
        {
            try
            {
                boost::this_thread::interruption_point();
                usleep(10000);
            }
            catch(boost::thread_interrupted&)
            {
                ROS_INFO("1£­dealGo2DockThread£­thread interrupt");
                gotDockMsg = 0;
                goto EXITLOOP;
            }

        }
        searchDock(1);
        ROS_INFO("start to get the dock msg.");
        while(ros::ok()&&(total_yaw<380))
        {
            int charge;
            n.getParam ("/battery/charge",charge);
            if(charge)
            {
                break;
            }
            if(gotDockMsg == 2)
                break;
            cmdvel_.linear.x = 0;
            cmdvel_.angular.z = speed;
            pub_vel.publish(cmdvel_);
            try
            {
                boost::this_thread::interruption_point();
                usleep(200000);
            }
            catch(boost::thread_interrupted&)
            {
                ROS_INFO("2£­dealGo2DockThread£­thread interrupt");
                gotDockMsg = 0;
                goto EXITLOOP;
            }
        }
EXITLOOP:
        searchDock(0);
        sub_dock.shutdown();
        cmdvel_.linear.x = 0;
        cmdvel_.angular.z = 0;
        pub_vel.publish(cmdvel_);
        sub_gyro.shutdown();
        return gotDockMsg;
    }

 /*   void checkBatteryVolume(void)
    {
        int volume;
        int charge;
        int cnt;
        int lowpowervolume = 10;
        int lowpowertime = 20;
        int charge_cnt = 0;
        n.getParam("/lowPowerVolume", lowpowervolume);
        n.getParam("/lowPowerTime", lowpowertime);
        ROS_WARN("lowpower standar is %d, and keep %d second", lowpowervolume, lowpowertime);
        while(ros::ok())
        {
            n.getParam ("/battery/volume",volume);
            n.getParam ("/battery/charge",charge);
            if(charge)
            {
                charge_cnt = 0;
            }
            if((search_dock == 0)&&(charge == 0))
            {
                if(volume < lowpowervolume)
                {
                    cnt ++;
                }
                else
                    cnt = 0;
            }
            else
            {
                cnt = 0;
            }
            sleep(1);
            if((cnt > lowpowertime)&&(charge_cnt < 4))
            {
                publishMasterT("any", "any", " ", 0);       //wakeup
                ROS_WARN("µçÁ¿²»×ã");
                publishMasterT("t_go2dock", "s_go2dock", "lowpower", 1);       //Æô¶¯³äµç
                cnt = 0;
                charge_cnt ++;
            }
        }

    }
*/

    void dealGo2DockThread(void)
    {
        int status;
		ROS_WARN("dealGo2DockThread");
        search_dock = 1;
        n.getParam ("/battery/charge",status);
        if(status)
        {
            ROS_WARN("i am charging");
        }
        else
        {
            ROS_WARN("i am looking for the dock!");
            if(turnAndSearchDock(0.4))
            {

                pubGo2Dock(0);
                usleep(500000);
                pubGo2Dock(1);

                while(ros::ok())
                {
                    try
                    {
                        boost::this_thread::interruption_point();
                        sleep(1);
                    }
                    catch(boost::thread_interrupted&)
                    {
                        ROS_INFO("3£­dealGo2DockThread£­thread interrupt");
                        break;
                    }

                    n.getParam ("/battery/charge",status);
                    if(status)
                    {
                        break;
                    }

                }
            }
            else
            {
                ROS_ERROR("Can not find the dock msg here!");
                ROS_WARN("¸½½üÕÒ²»µ½³äµç×ù£¬Í£Ö¹³äµç£¬Çë´øÎÒµ½³äµç×ù¸½½ü¡£");
            }
            pubGo2Dock(0);
        }
		ROS_WARN("Goodbye!!");
        search_dock = 0;
    }

	void pubMoving(float xx, float zz)
    {
		    geometry_msgs::Twist cmdvel_;
            cmdvel_.linear.x = xx;
            cmdvel_.angular.z = zz;
            pub_vel.publish(cmdvel_);
    }

    void turnTestThread(void)
	{
		ROS_WARN("turnTestThread");
		while(ros::ok())
		{
			try
			{
				boost::this_thread::interruption_point();
				usleep(500000);
			}
			catch(boost::thread_interrupted&)
			{
                ROS_INFO("thread interrupt");
				break;
			}

			pubMoving(0, 3);

		}
		pubMoving(0, 0);
		ROS_WARN("Goodbye!!");
	}
};




int main(int argc, char **argv)
{
    ros::init(argc, argv, "s_go2dock_node");
    ros::NodeHandle n;
	int model;
	model = atof(argv[1]);
    Go2DockNode gdn(n, model);
  //  smhn.creatThread(&IntroduceMyselfNode::test1);
   
    ros::spin();
    return 0;
}




