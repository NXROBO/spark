#!/usr/bin/env bash
PATH=/bin:/sbin:/usr/bin:/usr/sbin:/usr/local/bin:/usr/local/sbin:~/bin
export PATH

#=================================================
#	System Required: Ubuntu 14.04+
#	Description: Install ROS And Spark
#	Version: 1.0.21
#	Author: J.xiao
#	Site: http://www.nxrobo.com/
#=================================================


sh_ver="1.1.0"
filepath=$(cd "$(dirname "$0")"; pwd)
Green_font_prefix="\033[32m" && Red_font_prefix="\033[31m" && Green_background_prefix="\033[42;37m" && Red_background_prefix="\033[41;37m" && Font_color_suffix="\033[0m"
Info="${Green_font_prefix}[信息]${Font_color_suffix}"
Error="${Red_font_prefix}[错误]${Font_color_suffix}"
Tip="${Green_font_prefix}[注意]${Font_color_suffix}"
Separator_1="——————————————————————————————"

Version=$(lsb_release -r --short)
Codename=$(lsb_release -c --short)
OSDescription=$(lsb_release -d --short)
OSArch=$(uname -m)





#检查系统要求
check_sys(){
        if [[ "${Version}" == "14.04" ]]; then
                ROS_Ver="indigo"
        elif [[ "${Version}" == "16.04" ]]; then
                ROS_Ver="kinetic"
        elif [[ "${Version}" == "18.04" ]]; then
                ROS_Ver="melodic"
        else
                echo -e "${Error} SPARK不支持当前系统 ${OSDescription} !" && exit 1
        fi
}

#安装ROS完整版
install_ros_full(){
		sudo sh -c 'echo 1 > /proc/sys/net/ipv6/conf/all/disable_ipv6'
		sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
		sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116		
		sudo apt-get update
		sudo apt-get install -y ros-${ROS_Ver}-desktop-full
		sudo rosdep init
		rosdep update
		echo "source /opt/ros/${ROS_Ver}/setup.bash" >> ~/.bashrc
		source /opt/ros/${ROS_Ver}/setup.bash
		sudo apt-get install -y python-rosinstall python-rosinstall-generator python-wstool build-essential
}

#安装SPARK依赖库
install_spark_require(){
	echo -e "${Info} 准备安装SPARK相关驱动……"

	echo -e "${Info} 设置udev规则……"
	BASEPATH=$(cd `dirname $0`; pwd)
	sudo cp $BASEPATH/doc/rules/3ilidar-usb-serial.rules /etc/udev/rules.d/
	sudo cp $BASEPATH/doc/rules/spark-usb-serial.rules /etc/udev/rules.d/
	sudo cp $BASEPATH/doc/rules/orbbec-usb.rules /etc/udev/rules.d/556-orbbec-usb.rules
	sudo udevadm trigger

	echo -e "${Info} 安装所需要的依赖库……"
	#sudo sh -c 'echo 1 > /proc/sys/net/ipv6/conf/all/disable_ipv6'
	#sudo apt-get update
	sudo apt-get install -y ros-${ROS_Ver}-ecl ros-${ROS_Ver}-ecl-threads ros-${ROS_Ver}-rgbd-launch 
	sudo apt-get install -y ros-${ROS_Ver}-image-common
	sudo apt-get install -y ros-${ROS_Ver}-move-base-* 
	sudo apt-get install -y ros-${ROS_Ver}-serial
	sudo apt-get install -y python-pip python-sklearn libudev-dev
	sudo apt-get install -y ros-${ROS_Ver}-depthimage-to-laserscan ros-${ROS_Ver}-map-server ros-${ROS_Ver}-amcl ros-${ROS_Ver}-gmapping ros-${ROS_Ver}-navigation*
	sudo apt-get install -y ros-${ROS_Ver}-hector-mapping
	sudo apt-get install -y ros-${ROS_Ver}-frontier-exploration 
	sudo apt-get install -y libasound2-dev mplayer
	echo -e "${Info} 依赖库安装成功……"
}

#编译SPARK
install_spark(){
	if [[ "${Version}" == "18.04" ]]; then
		git checkout melodic-devel
	fi
	source /opt/ros/${ROS_Ver}/setup.bash
	catkin_make
	#catkin_make install
}


#完全安装
install_all(){
	install_ros_full
	install_spark_require
	install_spark
}

#远程设置
master_uri_setup(){
	eth_ip=`/sbin/ifconfig eth0|grep inet|awk '{print $2}'|awk -F: '{print $2}'`
	wlp2s_ip=`/sbin/ifconfig wlp2s0|grep inet|awk '{print $2}'|awk -F: '{print $2}'`
	wlan_ip=`/sbin/ifconfig wlan0|grep inet|awk '{print $2}'|awk -F: '{print $2}'`

	if [ $eth_ip ]; then
		echo -e "${Info}使用有线网络eth0" 
		local_ip=$eth_ip
	elif [ $wlp2s_ip ]; then
		echo -e "${Info}使用无线网络wlp2s0" 
	  	local_ip=$wlp2s_ip
	elif [ $wlan_ip ]; then
		echo -e "${Info}使用无线网络wlan0" 
	  	local_ip=$wlan_ip
	fi
	export ROS_HOSTNAME=$local_ip
	export ROS_MASTER_URI="http://${local_ip}:11311"
	echo -e "${Info}Using ROS MASTER at $ROS_MASTER_URI from $ROS_HOSTNAME"
}

#让机器人动起来
let_robot_go(){
	echo -e "${Info}" 
	echo -e "${Info}      让机器人动起来" 
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash

	echo -e "${Info}" 
	echo -e "${Info}    请在新的终端窗口操作"
	echo -e "${Info}键盘“wsad”分别对应“前后左右”"
	echo -e "${Info}                           " 
	echo -e "${Info}           w前进           "
	echo -e "${Info}    a左转         d右转    "
	echo -e "${Info}           s后退           " 
	echo -e "${Info}                           " 
	echo -e "${Info}    退出请输入：Ctrl + c    " 
	echo && stty erase ^? && read -p "按任意键开始：" 

	roslaunch spark_teleop teleop.launch
}


#远程（手机APP）控制SPARK
remote_control_robot(){
	master_uri_setup
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash
	echo -e "${Info}                  " 
	echo -e "${Info} 远程（手机APP）控制SPARK" 
	echo -e "${Info}" 
	echo -e "${Info}远程控制的APP地址: https://github.com/ros-autom/RobotCA/tree/kinetic/Release "
	echo -e "${Info}下载安装完成后，打开app，设置Master URI:http://${local_ip}:11311" 
	echo -e "${Info}接着就可以开始远程控制机器人了" 
	echo -e "${Info}退出请输入：Ctrl + c" 
	echo -e "${Info}" 
	echo && stty erase ^? && read -p "按任意键开始：" 

	roslaunch spark_teleop app_op.launch
}

#让SPARK跟着你走
people_follow(){
	echo -e "${Info}                  " 
	echo -e "${Info}让SPARK跟着你走" 
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash

	echo -e "${Info}                  " 
	echo -e "${Info}请站在SPARK的正前方，与SPARK保持一米左右的距离，然后走动"
	echo -e "${Info}                  " 
	echo -e "${Info}退出请输入：Ctrl + c " 
	echo -e "${Info}" 
	echo && stty erase ^? && read -p "按任意键开始：" 

	roslaunch spark_follower bringup.launch
}

#机械臂与摄像头匹对标定
cal_camera_arm(){
	echo -e "${Info}" 
	echo -e "${Info}机械臂与摄像头匹对标定" 
	ROSVER=`/usr/bin/rosversion -d`
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash

	echo -e "${Info}" 
	echo -e "${Info}请确定："
	echo -e "${Info}       A.摄像头已反向向下安装好。机械臂正常上电。"
	echo -e "${Info}       B.红色标定物已贴好在吸盘固定头正上方。"
	echo -e "${Info}       C.机械臂正常上电。" 
	echo -e "${Info}退出请输入：Ctrl + c " 
	echo -e "${Info}" 
	echo && stty erase ^? && read -p "按任意键开始：" 
	if [ $ROSVER = "kinetic" ]; then
		echo -e "${Info}It is kinetic." 
	  	roslaunch spark_carry_object spark_carry_cal_py3.launch
	elif [ $ROSVER = "indigo" ]; then
		echo -e "${Info}It is indigo." 
	  	roslaunch spark_carry_object spark_carry_cal_py2.launch
	elif [ $ROSVER = "melodic" ]; then
		echo -e "${Info}It is melodic." 
	  	roslaunch spark_carry_object spark_carry_cal_py3.launch
	fi
		
}


#让SPARK使用激光雷达进行导航
spark_navigation_2d(){
	echo -e "${Info}" 
	echo -e "${Info}让SPARK使用激光雷达进行导航" 
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash

	echo -e "${Info}" 
	echo -e "${Info}请注意："
	echo -e "${Info}       A.激光雷达已上电连接"
	echo -e "${Info}       B.导航正常启动后，点击‘2D Pose Estimate’后在地图上进行手动定位。"
	echo -e "${Info}       C.手动定位成功后，点击‘2D Nav Goal’后在地图上指定导航的目标点，机器人将进入自主导航。" 
	echo -e "${Info}退出请输入：Ctrl + c " 
	echo -e "${Info}" 
	echo && stty erase '^H' && read -p "按任意键开始：" 

	roslaunch spark_navigation amcl_demo_lidar_rviz.launch	
}
#让SPARK使用深度摄像头进行导航
spark_navigation_3d(){
	echo -e "${Info}" 
	echo -e "${Info}让SPARK使用深度摄像头进行导航" 
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash

	echo -e "${Info}" 
	echo -e "${Info}请注意："
	echo -e "${Info}       A.摄像头已连接"
	echo -e "${Info}       B.导航正常启动后，点击‘2D Pose Estimate’后在地图上进行手动定位。"
	echo -e "${Info}       C.手动定位成功后，点击‘2D Nav Goal’后在地图上指定导航的目标点，机器人将进入自主导航。" 
	echo -e "${Info}退出请输入：Ctrl + c " 
	echo -e "${Info}" 
	echo && stty erase ^? && read -p "按任意键开始：" 

	roslaunch spark_navigation amcl_demo_rviz.launch	
}
#让SPARK通过机械臂进行视觉抓取
spark_carry_obj(){
	echo -e "${Info}" 
	echo -e "${Info}让SPARK通过机械臂进行视觉抓取" 
	ROSVER=`/usr/bin/rosversion -d`
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash

	echo -e "${Info}" 
	echo -e "${Info}请确定："
	echo -e "${Info}       A.摄像头已反向向下安装好。机械臂正常上电。"
	echo -e "${Info}       B.红色标定物已贴好在吸盘固定头正上方。"
	echo -e "${Info}       C.机械臂正常上电。" 
	echo -e "${Info}退出请输入：Ctrl + c " 
	echo -e "${Info}" 
	echo && stty erase ^? && read -p "按任意键开始：" 
	if [ $ROSVER = "kinetic" ]; then
		echo -e "${Info}It is kinetic." 
	  	roslaunch spark_carry_object spark_carry_object_only_py3.launch 
	elif [ $ROSVER = "indigo" ]; then
		echo -e "${Info}It is indigo." 
	  	roslaunch spark_carry_object spark_carry_object_only_py2.launch 
	elif [ $ROSVER = "melodic" ]; then
		echo -e "${Info}It is melodic." 
	  	roslaunch spark_carry_object spark_carry_object_only_py3.launch 
	fi
	
}

#让SPARK使用激光雷达绘制地图(gmapping)
spark_build_map_2d(){
	echo -e "${Info}" 
	echo -e "${Info}让SPARK使用激光雷达绘制地图" 
	echo -e "${Info}" 
	echo -e "${Info}请选择SLAM的方式：
	  ${Green_font_prefix}1.${Font_color_suffix} gmapping
	  ${Green_font_prefix}2.${Font_color_suffix} hector
	  ${Green_font_prefix}3.${Font_color_suffix} frontier_exploration
	  ${Green_font_prefix}4.${Font_color_suffix} karto
	  ${Green_font_prefix}5.${Font_color_suffix} 退出请输入：Ctrl + c" 
	echo && stty erase ^? && read -p "请输入数字 [1-4]：" slamnum
	case "$slamnum" in
		1)
		SLAMTYPE="gmapping"
		;;
		2)
		SLAMTYPE="hector"
		;;
		3)
		SLAMTYPE="frontier_exploration"
		;;
		4)
		coming_soon
		exit
		#SLAMTYPE="karto"
		;;
		*)
		echo -e "${Error} 错误，默认使用gmapping"
		SLAMTYPE="gmapping"
		;;
	esac
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash

	echo -e "${Info}" 
	echo -e "${Info}    请在新的终端窗口操作"
	echo -e "${Info}键盘“wsad”分别对应“前后左右”"
	echo -e "${Info}                           " 
	echo -e "${Info}           w前进           "
	echo -e "${Info}    a左转         d右转    "
	echo -e "${Info}           s后退           " 
	echo -e "${Info}                           " 
	echo -e "${Info}退出请输入：Ctrl + c        " 
	echo -e "${Info}" 
	echo && stty erase ^? && read -p "按任意键开始：" 

	roslaunch spark_slam 2d_slam_teleop.launch slam_methods_tel:=${SLAMTYPE} 
	
}

#让SPARK使用深度摄像头绘制地图
spark_build_map_3d(){
	echo -e "${Info}" 
	echo -e "${Info}让SPARK使用深度摄像头绘制地图" 
	echo -e "${Info}" 
	echo -e "${Info}请选择SLAM的方式：
	  ${Green_font_prefix}1.${Font_color_suffix} gmapping
	  ${Green_font_prefix}2.${Font_color_suffix} hector
	  ${Green_font_prefix}3.${Font_color_suffix} frontier_exploration
	  ${Green_font_prefix}4.${Font_color_suffix} karto
	  ${Green_font_prefix}5.${Font_color_suffix} rtab_map
	  ${Green_font_prefix}6.${Font_color_suffix} 退出请输入：Ctrl + c" 
	echo && stty erase ^? && read -p "请输入数字 [1-5]：" slamnum
	case "$slamnum" in
		1)
		SLAMTYPE="gmapping"
		;;
		2)
		SLAMTYPE="hector"
		;;
		3)
		SLAMTYPE="frontier_exploration"
		;;
		4)
		coming_soon
		exit
		#SLAMTYPE="karto"
		;;
		5)
		coming_soon
		exit
		#SLAMTYPE="rtab_map"
		;;
		*)
		echo -e "${Error} 错误，默认使用gmapping"
		SLAMTYPE="gmapping"
		;;
	esac
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash

	echo -e "${Info}" 
	echo -e "${Info}    请在新的终端窗口操作"
	echo -e "${Info}键盘“wsad”分别对应“前后左右”"
	echo -e "${Info}                           " 
	echo -e "${Info}           w前进           "
	echo -e "${Info}    a左转         d右转    "
	echo -e "${Info}           s后退           " 
	echo -e "${Info}                           " 
	echo -e "${Info}退出请输入：Ctrl + c        " 
	echo -e "${Info}" 
	echo && stty erase ^? && read -p "按任意键开始：" 

	roslaunch spark_slam depth_slam_teleop.launch slam_methods_tel:=${SLAMTYPE} 
	
}

coming_soon(){
	echo -e "${Tip} coming_soon!" 
}


#printf
menu_status(){
	echo -e "${Tip} 当前系统版本 ${OSDescription} !" 
	ROSVER=`/usr/bin/rosversion -d`
	echo -e "${Tip} 当前ROS版本 ${ROSVER} !" 
}

check_sys
echo -e "  SPARK 一键安装管理脚本 ${Red_font_prefix}[v${sh_ver}]${Font_color_suffix}
  ---- J.xiao | www.nxrobo.com ----


  ${Green_font_prefix}0.${Font_color_suffix} 单独编译SPARK
————————————
  ${Green_font_prefix}1.${Font_color_suffix} 让机器人动起来
  ${Green_font_prefix}2.${Font_color_suffix} 远程（手机APP）控制SPARK
  ${Green_font_prefix}3.${Font_color_suffix} 让SPARK跟着你走
  ${Green_font_prefix}4.${Font_color_suffix} 让SPARK使用激光雷达绘制地图
  ${Green_font_prefix}5.${Font_color_suffix} 让SPARK使用深度摄像头绘制地图
  ${Green_font_prefix}6.${Font_color_suffix} 让SPARK使用激光雷达进行导航
  ${Green_font_prefix}7.${Font_color_suffix} 让SPARK使用深度摄像头进行导航
  ${Green_font_prefix}8.${Font_color_suffix} 机械臂与摄像头标定
  ${Green_font_prefix}9.${Font_color_suffix} 让SPARK通过机械臂进行视觉抓取
  ${Green_font_prefix}10.${Font_color_suffix} 其它
————————————
  ${Green_font_prefix}101.${Font_color_suffix} 完整安装
  ${Green_font_prefix}102.${Font_color_suffix} 单独安装ROS环境
  ${Green_font_prefix}103.${Font_color_suffix} 单独安装SPARK依赖

 "
menu_status
echo && stty erase ^? && read -p "请输入数字：" num
case "$num" in
	0)
	install_spark
	;;
	1)
	let_robot_go
	;;
	2)
	remote_control_robot
	;;
	3)
	people_follow
	;;
	4)
	spark_build_map_2d
	;;
	5)
	spark_build_map_3d
	;;
	6)
	spark_navigation_2d
	;;
	7)
	spark_navigation_3d
	;;
	8)
	cal_camera_arm
	;;
	9)
	spark_carry_obj
	;;
	10)
	menu_status
	;;
	101)
	install_all
	;;
	102)
	install_ros_full
	;;
	103)
	install_spark_require
	;;
	*)
	echo -e "${Error} 请输入正确的数字 "
	;;
esac
