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


sh_ver="1.0.21"
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


#让机器人动起来
let_robot_go(){
	echo -e "${Info}                  " 
	echo -e "${Info} 让机器人动起来" 
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash

	echo -e "${Info}                  " 
	echo -e "${Info}    请在新的终端窗口操作"
	echo -e "${Info}键盘“wsad”分别对应“前后左右”"
	echo -e "${Info}                           " 
	echo -e "${Info}           w前进           "
	echo -e "${Info}    a左转         d右转    "
	echo -e "${Info}           s后退           " 
	echo -e "${Info}                           " 
	echo -e "${Info}    退出请输入：Ctrl + c    " 
	echo && stty erase '^H' && read -p "按任意键开始：" 

	roslaunch spark_teleop teleop.launch
}


#远程（手机APP）控制SPARK
remote_control_robot(){
	echo -e "${Info}                  " 
	echo -e "${Info} 远程（手机APP）控制SPARK" 
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
	echo && stty erase '^H' && read -p "按任意键开始：" 

	roslaunch spark_teleop teleop.launch
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
	echo && stty erase '^H' && read -p "按任意键开始：" 

	roslaunch spark_follower bringup.launch
}

#机械臂与摄像头匹对标定
cal_camera_arm(){
	echo -e "${Info}" 
	echo -e "${Info}机械臂与摄像头匹对标定" 
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash

	echo -e "${Info}" 
	echo -e "${Info}请确定："
	echo -e "${Info}       A.摄像头已反向向下安装好。机械臂正常上电。"
	echo -e "${Info}       B.红色标定物已贴好在吸盘固定头正上方。"
	echo -e "${Info}       C.机械臂正常上电。" 
	echo -e "${Info}退出请输入：Ctrl + c " 
	echo -e "${Info}" 
	echo && stty erase '^H' && read -p "按任意键开始：" 

	roslaunch spark_carry_object spark_carry_cal.launch	
}

#让SPARK通过机械臂进行视觉抓取
spark_carry_obj(){
	echo -e "${Info}" 
	echo -e "${Info}让SPARK通过机械臂进行视觉抓取" 
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash

	echo -e "${Info}" 
	echo -e "${Info}请确定："
	echo -e "${Info}       A.摄像头已反向向下安装好。机械臂正常上电。"
	echo -e "${Info}       B.红色标定物已贴好在吸盘固定头正上方。"
	echo -e "${Info}       C.机械臂正常上电。" 
	echo -e "${Info}退出请输入：Ctrl + c " 
	echo -e "${Info}" 
	echo && stty erase '^H' && read -p "按任意键开始：" 

	roslaunch spark_carry_object spark_carry_object_only.launch 
	
}


#printf
menu_status(){
	echo -e "${Tip} 当前系统版本 ${OSDescription} !" 
}

check_sys
echo -e "  SPARK 一键安装管理脚本 ${Red_font_prefix}[v${sh_ver}]${Font_color_suffix}
  ---- J.xiao | www.nxrobo.com ----

  ${Green_font_prefix}1.${Font_color_suffix} 完整安装
  ${Green_font_prefix}2.${Font_color_suffix} 单独安装ROS环境
  ${Green_font_prefix}3.${Font_color_suffix} 单独安装SPARK依赖
  ${Green_font_prefix}4.${Font_color_suffix} 单独编译SPARK
————————————
  ${Green_font_prefix}5.${Font_color_suffix} 让机器人动起来
  ${Green_font_prefix}6.${Font_color_suffix} 远程（手机APP）控制SPARK
  ${Green_font_prefix}7.${Font_color_suffix} 让SPARK跟着你走
  ${Green_font_prefix}8.${Font_color_suffix} 让SPARK学会绘制地图
  ${Green_font_prefix}9.${Font_color_suffix} 让SPARK通过地图进行导航
  ${Green_font_prefix}10.${Font_color_suffix} 机械臂与摄像头标定
  ${Green_font_prefix}11.${Font_color_suffix} 让SPARK通过机械臂进行视觉抓取
  ${Green_font_prefix}12.${Font_color_suffix} 其它
 "
menu_status
echo && stty erase '^H' && read -p "请输入数字 [1-9]：" num
case "$num" in
	1)
	install_all
	;;
	2)
	install_ros_full
	;;
	3)
	install_spark_require
	;;
	4)
	install_spark
	;;
	5)
	let_robot_go
	;;
	6)
	menu_status
	;;
	7)
	people_follow
	;;
	8)
	menu_status
	;;
	9)
	menu_status
	;;
	10)
	cal_camera_arm
	;;
	11)
	spark_carry_obj
	;;
	12)
	menu_status
	;;
	13)
	menu_status
	;;
	*)
	echo -e "${Error} 请输入正确的数字 [1-15]"
	;;
esac
