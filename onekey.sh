#!/usr/bin/env bash
PATH=/bin:/sbin:/usr/bin:/usr/sbin:/usr/local/bin:/usr/local/sbin:~/bin
export PATH

#=================================================
#	System Required: Ubuntu 14.04+
#	Description: Install ROS And Spark
#	Version: 1.0.21
#	Author: J.xiao
#	Site: http://www.nxrobo.com/
#	SPARK技术讨论与反馈群：8346256
#=================================================
GAME_ENABLE="yes"
sh_ver="1.1.0"
filepath=$(cd "$(dirname "$0")"; pwd)
Green_font_prefix="\033[32m" && Red_font_prefix="\033[31m" && Green_background_prefix="\033[42;37m" && Red_background_prefix="\033[41;37m" && Font_color_suffix="\033[0m" && Yellow_font_prefix="\e[1;33m" && Blue_font_prefix="\e[0;34m"
Info="${Green_font_prefix}[信息]${Font_color_suffix}"
Error="${Red_font_prefix}[错误]${Font_color_suffix}"
Warn="${Yellow_font_prefix}[警告]${Font_color_suffix}"
Tip="${Green_font_prefix}[注意]${Font_color_suffix}"
Separator_1="——————————————————————————————"
password="spark"
Version=$(lsb_release -r --short)
Codename=$(lsb_release -c --short)
OSDescription=$(lsb_release -d --short)
OSArch=$(uname -m)
calibra_default="${filepath}/../.ros/camera_info"

calibration="calibration"
color_block="color_block"

TYPE_LIDAR=$(cat /opt/lidar.txt)
echo ${TYPE_LIDAR}
if [[ "${TYPE_LIDAR}" == "ydlidar_g2" ]]; then
	LIDARTYPE="ydlidar_g2"
elif [[ "${TYPE_LIDAR}" == "3iroboticslidar2" ]]; then
	LIDARTYPE="3iroboticslidar2"
else
	echo "暂不支持的雷达：${TYPE_LIDAR}，使用默认的杉川雷达运行"
	LIDARTYPE="3iroboticslidar2"
fi

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

#检查设备连接
check_dev(){
	#检查底盘
	if [ ! -n "$(lsusb -d 1a86:7523)" ]; then
		echo -e "${Error} 底盘没有正确连接，请确认正确连接！！"
	fi
	#检查机械臂
	if [ ! -n "$(lsusb -d 2341:0042)" ]; then
		echo -e "${Error} 机械臂没有正确连接，请确认正确连接！！"
	fi
	
	#检查摄像头
	check_camera
	#检查雷达
	check_lidar
}

check_camera_no_print(){
	#检查使用哪种设备
	if [ -n "$(lsusb -d 2bc5:0403)" ]; then
		CAMERATYPE="astrapro"
	fi
	if [ -n "$(lsusb -d 2bc5:0401)" ]; then
		CAMERATYPE="astra"
	fi
	if [ -n "$(lsusb -d 8086:0b07)" ]; then
		CAMERATYPE="d435"
	fi
}

#检查摄像头设备
check_camera(){

	camera_flag=0
	calibra_backup="/opt/nxrobo/camera_info"

	#检查使用哪种设备
	if [ -n "$(lsusb -d 2bc5:0403)" ]; then
		CAMERATYPE="astrapro"
		depthtolaser="/camera/depth/image_rect_raw"
		camera_flag=$[$camera_flag + 1]
	fi
	if [ -n "$(lsusb -d 2bc5:0401)" ]; then
		CAMERATYPE="astra"
		depthtolaser="/camera/depth/image_raw"
		camera_flag=$[$camera_flag + 1]
	fi
	if [ -n "$(lsusb -d 8086:0b07)" ]; then
		CAMERATYPE="d435"
		depthtolaser="/camera/depth/image_rect_raw"
		camera_flag=$[$camera_flag + 1]
	fi

	if [ $camera_flag -ge 2 ]; then
		echo -e "${Warn} 正在使用多个摄像头设备，请退出并拔掉其中一个再使用!"
		echo -e "${Warn} 退出请输入：Ctrl + c！"
	elif [ $camera_flag -eq 1 ]; then
		echo -e "${Info} 正在使用${CAMERATYPE}摄像头"
	elif [ $camera_flag -eq 0 ]; then
		echo -e "${Error} 没有找到摄像头，请确认摄像头已正确连接！！"
	fi	
	
	#根据摄像头类型检查标定文件和备份文件
	case $CAMERATYPE in
		astrapro) 
			#检查是否有默认文件夹和备份文件夹
			if [ ! -d "$calibra_default" ]; then
				mkdir -p "$calibra_default"
			fi

			if [ ! -d "$calibra_backup" ]; then
				echo -e "${Info} 创建标定文件备份文件夹..."
				sudo mkdir  -p "$calibra_backup"
			fi
			#如果没有就拷贝一份
			if [ ! -f "$calibra_backup/camera.yaml" ]||[ ! -f "$calibra_backup/depth_Astra_Orbbec.yaml" ]; then
				if [ -f "$calibra_default/camera.yaml" ]&&[ -f "$calibra_default/depth_Astra_Orbbec.yaml" ]; then
					echo -e "${Info} 备份标定文件..."
					sudo cp -r "$calibra_default/." "$calibra_backup/."
				elif [ ! -f "$calibra_default/camera.yaml" ]||[ ! -f "$calibra_default/depth_Astra_Orbbec.yaml" ]; then
					echo -e "${Warn} 标定文件缺少其中一个，请确认rgb和depth标定文件都存在"
				fi
			elif [ -f "$calibra_backup/camera.yaml" ]&&[ -f "$calibra_backup/depth_Astra_Orbbec.yaml" ]; then
				if [ ! -f "$calibra_default/camera.yaml" ]; then
					echo -e "${Info} 缺少彩色摄像头标定文件，已拷贝"
					cp "$calibra_backup/camera.yaml" $calibra_default
				fi
				if [ ! -f "$calibra_default/depth_Astra_Orbbec.yaml" ]; then
					echo -e "${Info} 缺少深度摄像头标定文件，已拷贝"
					cp "$calibra_backup/depth_Astra_Orbbec.yaml" $calibra_default	
				fi
			fi
			;;
		astra)
			#astra自带标定文件			
			#如果检查到有标定文件就删除
			if [ -d "$calibra_default" ]; then
				rm -rf "$calibra_default"
			fi
			;;
	esac

}

#检查雷达设备
check_lidar(){

	lidar_flag=0

	#检查使用哪种设备
	if [ -n "$(lsusb -d 10c4:ea60)" ]; then
		lidar_flag=$[$lidar_flag + 1]
	fi



	if [ $lidar_flag -ge 2 ]; then
		echo -e "${Warn} 正在使用多个雷达设备，请退出并拔掉其中一个再使用!"
		echo -e "${Warn} 退出请输入：Ctrl + c！"
	elif [ $lidar_flag -eq 1 ]; then
		echo -e "${Info} 正在使用${LIDARTYPE}雷达"
	elif [ $lidar_flag -eq 0 ]; then
		echo -e "${Error} 没有找到雷达，请确认雷达已正确连接！！"
	fi	
}


#安装ROS完整版
install_ros_full(){
		sudo sh -c 'echo 1 > /proc/sys/net/ipv6/conf/all/disable_ipv6'
		sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
		sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654		
		sudo apt-get update
		sudo apt-get install -y ros-${ROS_Ver}-desktop-full
		sudo rosdep init
		rosdep update
		echo "source /opt/ros/${ROS_Ver}/setup.bash" >> ~/.bashrc
		source /opt/ros/${ROS_Ver}/setup.bash
		sudo apt-get install -y python-rosinstall python-rosinstall-generator python-wstool build-essential
}

#检测是否需要安装完整版
check_install_ros_full(){
	if [ -f "/usr/bin/rosversion" ]; then
		ROSVER=`/usr/bin/rosversion -d`
		if [ $ROSVER ]; then
			echo -e "${Tip} 检测到当前系统已安装了ROS的${ROSVER}版本!" 
			echo && stty erase ^? && read -p "请选择是否继续安装？ y/n：" choose
			if [[ "${choose}" == "y" ]]; then
				echo -e "${Info}准备安装ROS系统！" 
			else
				exit
			fi
		fi
	fi
	install_ros_full 
}

#安装SPARK依赖库
install_spark_require(){
	echo -e "${Info} 准备安装SPARK相关驱动……"
	echo 'deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main' | sudo tee /etc/apt/sources.list.d/realsense-public.list
	sudo apt-key adv --keyserver keys.gnupg.net --recv-key 6F3EFCDE
	if [[ "${ROS_Ver}" == "kinetic" ]]; then
		sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
	elif [[ "${ROS_Ver}" == "melodic" ]]; then
		sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
	fi

	echo -e "${Info} 设置udev规则……"
	BASEPATH=$(cd `dirname $0`; pwd)
	sudo cp $BASEPATH/doc/rules/3ilidar-usb-serial.rules /etc/udev/rules.d/
	sudo cp $BASEPATH/doc/rules/uarm-usb-serial.rules /etc/udev/rules.d/
	sudo cp $BASEPATH/doc/rules/spark-usb-serial.rules /etc/udev/rules.d/
	sudo cp $BASEPATH/doc/rules/orbbec-usb.rules /etc/udev/rules.d/556-orbbec-usb.rules
	sudo udevadm trigger

	echo -e "${Info} 安装所需要的依赖库……"
	#sudo sh -c 'echo 1 > /proc/sys/net/ipv6/conf/all/disable_ipv6'
	sudo apt-get update
	sudo apt-get install -y ros-${ROS_Ver}-ecl ros-${ROS_Ver}-ecl-threads ros-${ROS_Ver}-rgbd-launch 
	sudo apt-get install -y ros-${ROS_Ver}-image-common
	sudo apt-get install -y ros-${ROS_Ver}-move-base-* 
	sudo apt-get install -y ros-${ROS_Ver}-serial
	sudo apt-get install -y ros-${ROS_Ver}-libuvc-camera
	sudo apt-get install -y python-pip python-sklearn libudev-dev
	sudo apt-get install -y ros-${ROS_Ver}-depthimage-to-laserscan ros-${ROS_Ver}-map-server ros-${ROS_Ver}-amcl ros-${ROS_Ver}-gmapping ros-${ROS_Ver}-navigation ros-${ROS_Ver}-navigation-stage ros-${ROS_Ver}-navigation-layers ros-${ROS_Ver}-navigation-tutorials
	sudo apt-get install -y ros-${ROS_Ver}-hector-mapping
	sudo apt-get install -y ros-${ROS_Ver}-frontier-exploration
	sudo apt-get install -y ros-${ROS_Ver}-cartographer*
	sudo apt-get install -y ros-${ROS_Ver}-rtabmap-ros 
	sudo apt-get install -y ros-${ROS_Ver}-slam-karto
	sudo apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg
	sudo apt-get install -y ros-${ROS_Ver}-ddynamic-reconfigure
	sudo apt-get install -y libasound2-dev mplayer
	sudo apt-get install -y ros-${ROS_Ver}-usb-cam ros-${ROS_Ver}-openni2-launch
	sudo apt-get install -y python-dev python-virtualenv
	

        echo -e "${Info} 安装tensorflow依赖库……"
	sudo apt install python-testresources
	pip install setuptools -i https://pypi.douban.com/simple/
	sudo pip install --upgrade setuptools -i https://pypi.douban.com/simple/
	pip install virtualenv -i https://pypi.douban.com/simple/
	sudo pip install --upgrade virtualenv -i https://pypi.douban.com/simple/
	virtualenv --system-site-packages $BASEPATH/tensorflow
	source $BASEPATH/tensorflow/bin/activate
	easy_install -U pip
	pip install --upgrade tensorflow==1.5.0 -i https://pypi.douban.com/simple/
	source $BASEPATH/tensorflow/bin/activate

	echo -e "${Info} 安装语音依赖库……"
        sudo apt-get install -y swig portaudio19-dev
        #sudo easy_install -U pip
	sudo pip install pocketsphinx webrtcvad pyaudio wxpy

	echo -e "${Info} 依赖库安装成功……"
}

#安装intel_movidius的相关驱动和程序
install_intel_movidius(){
	BASEPATH=$(cd `dirname $0`; pwd)

	if [[ ! -d "$BASEPATH/src/3rd_app/intel/ncappzoo" ]] || [[ ! -d "$BASEPATH/src/3rd_app/intel/ros_intel_movidius_ncs" ]]  || [[ ! -d "/opt/movidius" ]] ; then
		echo && stty erase ^? && read -p "检测到未安装INTEL　MOVIDIUS的相关驱动和程序，是否现在安装y/n?" yorn 
		if [[ "${yorn}" == "y" ]]; then
			echo -e "${Info} 准备安装intel movidius的相关驱动和代码…… "
			echo -e "${Info} 安装过程中可能会花费挺长时间的。请耐心等待！"
			
			cd $BASEPATH
			mkdir $BASEPATH/src/3rd_app/intel
			cd $BASEPATH/src/3rd_app/intel
			echo -e "${Info} git clone https://github.com/movidius/ncsdk"
			git clone https://github.com/movidius/ncsdk
			echo -e "${Info} git clone https://github.com/movidius/ncappzoo"
			git clone https://github.com/movidius/ncappzoo
			echo -e "${Info} git clone https://github.com/intel/object_msgs"
			git clone https://github.com/intel/object_msgs
			echo -e "${Info} git clone https://github.com/intel/ros_intel_movidius_ncs.git"
			git clone https://github.com/intel/ros_intel_movidius_ncs.git
			cd $BASEPATH/src/3rd_app/intel/ncsdk
			make install
			make examples
			echo -e "${Info} sudo ln -s $BASEPATH/src/3rd_app/intel/ncappzoo /opt/movidius/ncappzoo"
			sudo ln -s $BASEPATH/src/3rd_app/intel/ncappzoo /opt/movidius/ncappzoo
			cd $BASEPATH/src/3rd_app/intel/ros_intel_movidius_ncs
			git checkout master
	
			cp $BASEPATH/src/3rd_app/intel/ros_intel_movidius_ncs/data/labels/* /opt/movidius/ncappzoo/data/ilsvrc12/

		#	AlexNet
			echo -e "${Info} compile NCS graph--AlexNet"
			cd /opt/movidius/ncappzoo/caffe/AlexNet
			make
		#	GoogleNet
			echo -e "${Info} compile NCS graph--GoogleNet"
			cd /opt/movidius/ncappzoo/caffe/GoogLeNet
			make
		#	SqueezeNet
			echo -e "${Info} compile NCS graph--SqueezeNet"
			cd /opt/movidius/ncappzoo/caffe/SqueezeNet
			make
		#	Inception_V1
			echo -e "${Info} compile NCS graph--Inception_V1"
			cd /opt/movidius/ncappzoo/tensorflow/inception_v1/
			make
		#	Inception_V2
			echo -e "${Info} compile NCS graph--Inception_V2"
			cd /opt/movidius/ncappzoo/tensorflow/inception_v2/
			make
		#	Inception_V3
			echo -e "${Info} compile NCS graph--Inception_V3"
			cd /opt/movidius/ncappzoo/tensorflow/inception_v3/
			make
		#	Inception_V4
			echo -e "${Info} compile NCS graph--Inception_V4"
			cd /opt/movidius/ncappzoo/tensorflow/inception_v4/
			make
		#	MobileNet
			echo -e "${Info} compile NCS graph--MobileNet"
			cd /opt/movidius/ncappzoo/tensorflow/mobilenets/
			make

		#	MobileNet_SSD
			echo -e "${Info} compile NCS graph--MobileNet_SSD"
			cd /opt/movidius/ncappzoo/caffe/SSD_MobileNet
			make	
		#	TinyYolo
			echo -e "${Info} compile NCS graph--TinyYolo"
			cd /opt/movidius/ncappzoo/caffe/TinyYolo
			make

			echo -e "${Info} finish compiling..."	
			echo -e "${Info} start to catkin_make..."
	
			cd $BASEPATH
			catkin_make
			echo -e "${Info} finsh catkin_make..."
		else
			echo -e "${Info} 取消安装."
			exit
		fi
	fi

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
	check_install_ros_full
	install_spark_require
	install_spark
}

#远程设置
master_uri_setup(){
	eth_ip=`/sbin/ifconfig eth0|grep inet|awk '{print $2}'|awk -F: '{print $2}'`
	wlp1s_ip=`/sbin/ifconfig wlp1s0|grep inet|awk '{print $2}'|awk -F: '{print $2}'`
	wlp2s_ip=`/sbin/ifconfig wlp2s0|grep inet|awk '{print $2}'|awk -F: '{print $2}'`
	wlan_ip=`/sbin/ifconfig wlan0|grep inet|awk '{print $2}'|awk -F: '{print $2}'`
        enp3s_ip=`/sbin/ifconfig enp3s0|grep inet|awk '{print $2}'|awk -F: '{print $2}'`
	wlp0s_ip=`/sbin/ifconfig wlp0s20f3|grep inet|awk '{print $2}'|awk -F: '{print $2}'`
	if [ $eth_ip ]; then
		echo -e "${Info}使用有线网络eth0" 
		local_ip=$eth_ip
	elif [ $wlp1s_ip ]; then
		echo -e "${Info}使用无线网络wlp1s0" 
	  	local_ip=$wlp1s_ip
	elif [ $wlp2s_ip ]; then
		echo -e "${Info}使用无线网络wlp2s0" 
	  	local_ip=$wlp2s_ip
	elif [ $wlan_ip ]; then
		echo -e "${Info}使用无线网络wlan0" 
	  	local_ip=$wlan_ip
        elif [ $enp3s_ip ]; then
                echo -e "${Info}使用无线网络enp3s0" 
                local_ip=$enp3s_ip	
        elif [ $wlp0s_ip ]; then
                echo -e "${Info}使用无线网络wlp0s20f3" 
                local_ip=$wlp0s_ip	
	fi
	export ROS_HOSTNAME=$local_ip
	export ROS_MASTER_URI="http://${local_ip}:11311"
	echo -e "${Info}Using ROS MASTER at ${Red_font_prefix}$ROS_MASTER_URI${Font_color_suffix} from ${Red_font_prefix}$ROS_HOSTNAME${Font_color_suffix}"
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
	echo && stty erase ^? && read -p "按回车键（Enter）开始：" 

	roslaunch spark_teleop teleop.launch camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE}
}


#远程（手机APP）控制SPARK
remote_control_robot(){
	master_uri_setup
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash
	echo -e "${Info}                  " 
	echo -e "${Info} 远程（手机APP）控制SPARK" 
	echo -e "${Info}" 
	echo -e "${Info}远程控制的APP地址:${Yellow_font_prefix}https://raw.githubusercontent.com/iamzhuang/RobotCA/kinetic/Release/control_app-debug.apk"${Font_color_suffix}
	echo -e "${Info}下载安装完成后，打开app，设置Master URI:${Red_font_prefix}http://${local_ip}:11311${Font_color_suffix}" 
	echo -e "${Info}接着就可以开始远程控制机器人了" 
	echo -e "${Info}退出请输入：Ctrl + c" 
	echo -e "${Info}" 
	echo && stty erase ^? && read -p "按回车键（Enter）开始：" 

	roslaunch spark_teleop app_op.launch camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE}
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
	echo && stty erase ^? && read -p "按回车键（Enter）开始：" 

	roslaunch spark_follower bringup.launch camera_type_tel:=${CAMERATYPE} 
}


#语音控制SPARK移动
voice_nav(){
	echo -e "${Info}" 
	echo -e "${Info}语音控制SPARK移动" 
	echo -e "${Info}" 
	echo -e "${Info}请选择语音的方式：
	  ${Green_font_prefix}1.${Font_color_suffix} 本地语音识别(识别准确度低,已不推荐使用,仅支持英文)
	  ${Green_font_prefix}2.${Font_color_suffix} 在线语音识别(需要互联网连接,只支持中文)
	  ${Green_font_prefix}3.${Font_color_suffix} 退出请输入：Ctrl + c" 
	echo && stty erase ^? && read -p "请输入数字 [1-3]：" asrnum
	case "$asrnum" in
		1)
		ASRTYPE="voice_nav.launch"
		;;
		2)
		ASRTYPE="baidu_nav.launch"
		;;
		*)
		echo -e "${Error} 错误，默认使用在线方式"
		ASRTYPE="baidu_nav.launch"
		;;
	esac
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash

	echo -e "${Info}                  " 
	echo -e "${Info}请确认连接线并设定正确的默认麦克风"
	echo -e "${Info}                  " 
	echo -e "${Info}退出请输入：Ctrl + c " 
	echo -e "${Info}" 
	echo && stty erase ^? && read -p "按回车键（Enter）开始：" 

	roslaunch spark_voice ${ASRTYPE} camera_type_tel:=${CAMERATYPE}

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
	echo -e "${Info}       B.${Red_font_prefix}红色${Font_color_suffix}标定物已贴好在吸盘固定头正上方。"
	echo -e "${Info}       C.机械臂正常上电。" 
	echo -e "${Info}退出请输入：Ctrl + c " 
	echo -e "${Info}" 
	echo && stty erase ^? && read -p "按回车键（Enter）开始：" 
	if [ $ROSVER = "kinetic" ]; then
		echo -e "${Info}It is kinetic." 
	  	roslaunch spark_carry_object spark_carry_cal_cv3.launch camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE}
	elif [ $ROSVER = "indigo" ]; then
		echo -e "${Info}It is indigo." 
	  	roslaunch spark_carry_object spark_carry_cal_cv2.launch camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE}
	elif [ $ROSVER = "melodic" ]; then
		echo -e "${Info}It is melodic." 
	  	roslaunch spark_carry_object spark_carry_cal_cv3.launch camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE}
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
	echo && stty erase '^H' && read -p "按回车键（Enter）开始：" 

	roslaunch spark_navigation amcl_demo_lidar_rviz.launch camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE}
}
#让SPARK使用深度摄像头进行导航
spark_navigation_3d(){
	echo -e "${Info}" 
	echo -e "${Info}让SPARK使用深度摄像头进行导航" 
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash
	echo -e "${Info}请选择导航方式："
	echo -e "${Info}1.使用2D地图"
	echo -e "${Info}2.使用rtab_map地图"
	echo && stty erase ^? && read -p "请输入数字 [1-2]：" slamnum

	echo -e "${Info}" 
	echo -e "${Info}请注意："
	echo -e "${Info}A.摄像头已连接"
	case "$slamnum" in
		1)
		SLAMTYPE="2d"
		echo -e "${Info}B.导航正常启动后，点击‘2D Pose Estimate’后在地图上进行手动定位。"
		;;
		2)
		SLAMTYPE="rtab_map"
		echo -e "${Info}B.把机器人放到原来建图的原点。导航正常启动后，如需查看原来建立的３Ｄ地图，点击rviz的Display->Rtabmap cloud->Download map加载３Ｄ地图。"
		;;
		*)
		echo -e "${Error} 错误，默认使用2D地图"
		SLAMTYPE="2d"
		echo -e "${Info}B.导航正常启动后，点击‘2D Pose Estimate’后在地图上进行手动定位。"
		;;
	esac

	echo -e "${Info}C.手动定位成功后，点击‘2D Nav Goal’后在地图上指定导航的目标点，机器人将进入自主导航。" 
	echo -e "${Info}退出请输入：Ctrl + c " 
	echo -e "${Info}" 
	echo && stty erase ^? && read -p "按回车键（Enter）开始：" 
	if [[ "${SLAMTYPE}" == "2d" ]]; then
		roslaunch spark_navigation amcl_demo_rviz.launch camera_type_tel:=${CAMERATYPE} depthtolaser:=${depthtolaser}
	else
		roslaunch spark_rtabmap spark_rtabmap_nav.launch camera_type_tel:=${CAMERATYPE} depthtolaser:=${depthtolaser}
	fi	
}

#深度学习
spark_DeepLearn(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/tensorflow/bin/activate
	source ${PROJECTPATH}/devel/setup.bash	
	echo -e "${Info}运行深度学习的方式：
	1.软件tensorflow
	2.硬件计算棒（需将intel movidius接在spark的${Blue_font_prefix}蓝色${Font_color_suffix}的USB3.0上）"
	echo && stty erase ^? && read -p "请选择 1 或 2 ：" chnum
 	case "$chnum" in
		1)
		roslaunch tensorflow_object_detector object_detect.launch camera_type_tel:=${CAMERATYPE}		
		;;
		2)
		spark_intel_movidius
		;;
		*)
		echo -e "${Error} 错误，将运行默认的软件tensorflow"
		roslaunch tensorflow_object_detector object_detect.launch camera_type_tel:=${CAMERATYPE}		
		;;
	esac
}

#intel_movidius的功能列表
intel_movidius_list(){
echo -e "
  请根据右侧的功能说明选择相应的序号。

  ${Green_font_prefix} NO.${Font_color_suffix} |   CNN Model   | Framework  |
  ${Green_font_prefix}  1.${Font_color_suffix} |    AlexNet    |   Caffe    |
  ${Green_font_prefix}  2.${Font_color_suffix} |   GoogleNet   |   Caffe    |
  ${Green_font_prefix}  3.${Font_color_suffix} |   SqueezeNet  |   Caffe    |
  ${Green_font_prefix}  4.${Font_color_suffix} |  Inception_v1 | tensorflow |
  ${Green_font_prefix}  5.${Font_color_suffix} |  Inception_v2 | tensorflow |
  ${Green_font_prefix}  6.${Font_color_suffix} |  Inception_v3 | tensorflow |
  ${Green_font_prefix}  7.${Font_color_suffix} |  Inception_v4 | tensorflow |
  ${Green_font_prefix}  8.${Font_color_suffix} |   MobileNet   | tensorflow |
  ${Green_font_prefix}  9.${Font_color_suffix} |  MobileNetSSD |   Caffe    |
  ${Green_font_prefix} 10.${Font_color_suffix} |    TinyYolo   |   Caffe    |"

}

#运行intel_movidius的相关功能
spark_intel_movidius(){
	intel_device=`/usr/bin/lsusb |grep 03e7:|awk '{print $6}'`
	if [ $intel_device ]; then
                echo -e "${Info}检测到INTEL MOVIDIUS神经计算棒设备，请确定该设备已插入到SPARK主机的USB3.0的接口（${Blue_font_prefix}蓝色${Font_color_suffix}USB接口）。" 
		install_intel_movidius
		echo -e "${Info}请选择下表的相关功能："
		intel_movidius_list
		echo && stty erase ^? && read -p "请输入数字 [1-10]：" chnum
 		case "$chnum" in
			1)
			roslaunch app_shell intel_movidius_classification.launch cnn_model:=alexnet
			;;
			2)
			roslaunch app_shell intel_movidius_classification.launch cnn_model:=googlenet
			;;
			3)
			roslaunch app_shell intel_movidius_classification.launch cnn_model:=squeezenet
			;;
			4)
			roslaunch app_shell intel_movidius_classification.launch cnn_model:=inception_v1
			;;
			5)
			roslaunch app_shell intel_movidius_classification.launch cnn_model:=inception_v2
			;;
			6)
			roslaunch app_shell intel_movidius_classification.launch cnn_model:=inception_v3
			;;
			7)
			roslaunch app_shell intel_movidius_classification.launch cnn_model:=inception_v4
			;;
			8)
			roslaunch app_shell intel_movidius_classification.launch cnn_model:=mobilenet
			;;
			9)
			roslaunch app_shell intel_movidius_detection.launch cnn_model:=mobilenetssd
			;;
			10)
			roslaunch app_shell intel_movidius_detection.launch cnn_model:=tinyyolo_v1 
			;;

			*)
			echo -e "${Error} 错误，将运行默认的mobilenetssd功能"
			roslaunch app_shell intel_movidius_detection.launch cnn_model:=mobilenetssd
			;;
		esac
	else
                echo -e "${Info}没检测到已插入intel movidius神经计算棒。请先插入相关设备到${Blue_font_prefix}蓝色${Font_color_suffix}的USB3.0接口。" 
		exit	
	fi
}

#spark_test_mode
spark_test_mode(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/tensorflow/bin/activate
	source ${PROJECTPATH}/devel/setup.bash	
	echo -e "${Info}老化测试程序，请将机器人放在一个50X50CM的方格中间进行测试，请选择：
	1.执行5分钟的检测功能；
        2.执行老化检测功能，直至断电。"
        echo -e "${Info}退出请输入：Ctrl + c " 
	echo && stty erase ^? && read -p "请选择 1 或 2 ：" chnum
 	case "$chnum" in
		1)
		roslaunch spark_test all_run_test_st.launch camera_type_tel:=${CAMERATYPE}	
		;;
		2)
		roslaunch spark_test all_run_test.launch camera_type_tel:=${CAMERATYPE}	
		;;
		*)
		echo -e "${Error} 退出!"	
		;;
	esac
}
#让SPARK通过机械臂进行视觉抓取
spark_carry_obj(){
	echo -e "${Info}" 
	echo -e "${Info}让SPARK通过机械臂进行视觉抓取" 
	ROSVER=`/usr/bin/rosversion -d`
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash
	echo -e "${Info}请选择移动的方式：
	  ${Green_font_prefix}1.${Font_color_suffix} 固定位置移动
	  ${Green_font_prefix}2.${Font_color_suffix} 手动地图指定位置导航(未完成。。)
	  ${Green_font_prefix}3.${Font_color_suffix} 退出请输入：Ctrl + c" 
	echo && stty erase ^? && read -p "请输入数字 [1-2]：" armnum
	case "$armnum" in
		1)
		MOVETYPE="fix"
		;;
		2)
		MOVETYPE="slam"
		;;
		*)
		echo -e "${Error} 错误，默认使用固定位置移动"
		MOVETYPE="fix"
		;;
	esac
	echo -e "${Info}" 
	echo -e "${Info}请确定："
	echo -e "${Info}       A.摄像头已反向向下安装好。机械臂正常上电。"
	echo -e "${Info}       B.机械臂正常上电。" 
	echo -e "${Info}       C.准备好可吸附的${Blue_font_prefix}蓝色${Font_color_suffix}物品。" 
	echo -e "${Info}退出请输入：Ctrl + c " 
	echo -e "${Info}" 
	echo && stty erase ^? && read -p "按回车键（Enter）开始：" 
	if [ $ROSVER = "kinetic" ]; then
		echo -e "${Info}It is kinetic." 
	  	roslaunch spark_carry_object spark_carry_object_only_cv3.launch camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE}
	elif [ $ROSVER = "indigo" ]; then
		echo -e "${Info}It is indigo." 
	  	roslaunch spark_carry_object spark_carry_object_only_cv2.launch camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE}
	elif [ $ROSVER = "melodic" ]; then
		echo -e "${Info}It is melodic." 
	  	roslaunch spark_carry_object spark_carry_object_only_cv3.launch camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE}
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
		SLAMTYPE="karto"
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
	echo && stty erase ^? && read -p "按回车键（Enter）开始：" 

	roslaunch spark_slam 2d_slam_teleop.launch slam_methods_tel:=${SLAMTYPE} camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE}
	
}
#让SPARK去充电
spark_dock(){

	echo -e "${Info}" 
	echo -e "${Info} 让SPARK去充电。注：只适用于带充电座的SPARK使用！" 
	echo -e "${Info} 退出请输入：Ctrl + c        "
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash
	echo && stty erase ^? && read -p "按回车键（Enter）继续：" 
	echo && stty erase ^? && read -p "请再次确定是否有带充电座 y/n：" choose
	if [[ "${choose}" == "y" ]]; then
		roslaunch spark_test go2dock.launch
	else
		return
	fi

}


spark_test(){

	echo -e "${Info}" 
	echo -e "${Info}      SPARK test" 
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash

	echo && stty erase ^? && read -p "按回车键（Enter）开始：" 

	roslaunch spark_test turn_test.launch
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
		SLAMTYPE="karto"
		;;
		5)
		SLAMTYPE="rtab_map"
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
	echo && stty erase ^? && read -p "按回车键（Enter）开始：" 

	if [[ "${SLAMTYPE}" == "rtab_map" ]]; then
		echo -e "${Tip}" 
		echo -e "${Tip}现在使用rtab_map建图，将会删除之前保存的地图，选择‘y’继续建图，其它键直接退出。" 
		echo -e "${Tip}" 
		echo && stty erase ^? && read -p "请选择是否继续y/n：" choose
		if [[ "${choose}" == "y" ]]; then
                	roslaunch spark_rtabmap spark_rtabmap_teleop.launch camera_type_tel:=${CAMERATYPE} depthtolaser:=${depthtolaser}
		else
			return
		fi
        else
		roslaunch spark_slam depth_slam_teleop.launch slam_methods_tel:=${SLAMTYPE} camera_type_tel:=${CAMERATYPE} depthtolaser:=${depthtolaser}
	fi
	
}

#给摄像机做标定
calibrate_camera(){
	echo -e "${Info} "
	echo -e "${Info}给摄像头做标定"
	echo -e "${Info} "
	echo -e "${Info}请选择：
	  ${Green_font_prefix}1.${Font_color_suffix} 彩色(rgb)摄像头标定
	  ${Green_font_prefix}2.${Font_color_suffix} 深度(depth/ir)摄像头标定
	  ${Green_font_prefix}3.${Font_color_suffix} 退出请输入：Ctrl + c"
	echo && stty erase ^? && read -p "请输入数字 [1-2]：" calinum
		case "$calinum" in
		1)
		CALIBRATETYPE="rgb"
		;;
		2)
		CALIBRATETYPE="depth"
		;;
		*)
		echo -e "${Error} 错误，退出" && exit
		;;
	esac
	echo -e "${Info}" 
	echo -e "${Info}标定步骤                                  					"
	echo -e "${Info}1.准备格子大小为20x20mm的9x7的黑白棋盘（在doc文件夹中可以找到）                    "
	echo -e "${Info}                            							" 	
	echo -e "${Info}2.倾斜或移动棋盘格子使X，Y，Size，Skew指标变成${Green_font_prefix}绿色          	"
	echo -e "${Info}                            							" 
	echo -e "${Info}3.点击窗口中的CALIBRATE进行标定                            			"
	echo -e "${Info}${Yellow_font_prefix}注意：${Font_color_suffix}点击后图像窗口将会卡顿！请耐心等候至新终端窗口出现标定矩阵数据！  		"
	echo -e "${Info}                            							" 	 
	echo -e "${Info}4.点击SAVE按钮保存标定文件                          				" 
	echo -e "${Info}                       " 
	echo -e "${Info}标定文件储存在/tmp/calibrationdata.tar.gz中 	        			"
	echo -e "${Info}请把压缩文件中的yaml格式文件拷贝至/home/spark/.ros/camera_info中  		"
	echo -e "${Info}								"
	echo -e "${Info}${Yellow_font_prefix}注意：${Font_color_suffix}请把yaml文件命名为当以下两种"
	echo -e "${Info}rgb标定：camera.yaml         depth标定：depth_Astra_Orbbec.yaml"
	echo -e "${Info}         "
	echo -e "${Info}退出请输入：Ctrl + c                                             		" 
	echo -e "${Info}										" 
	echo && stty erase ^? && read -p "按回车键（Enter）开始：" 

	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash
	roslaunch camera_calibrator camera_calibrate.launch calibrate_type:=${CALIBRATETYPE} camera_type_tel:=${CAMERATYPE}

}

#hsv范围值查找
spark_hsv_detection(){
	echo -e "${Info}"
	echo -e "${Info}hsv值自动查找"
	ROSVER=`/usr/bin/rosversion -d`
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash

	echo -e "${Info}"
	echo -e "${Info}请确定："
	echo -e "${Info}       A.摄像头已反向向下安装好。机械臂正常上电。"
	echo -e "${Info}       B.${Red_font_prefix}红色${Font_color_suffix}标定物已贴好在吸盘固定头正上方。"
	echo -e "${Info}       C.机械臂正常上电。"
	echo -e "${Info}退出请输入：Ctrl + c "
	echo -e "${Info}"

	echo -e "${Info}请选择比赛方式：
	  ${Green_font_prefix}1.${Font_color_suffix} 获取吸盘上方hsv值
	  ${Green_font_prefix}2.${Font_color_suffix} 获取颜色块hsv值,需要将颜色块放在图像的矩形框中
	  ${Green_font_prefix}3.${Font_color_suffix} 退出请输入：Ctrl + c"
	echo && stty erase ^? && read -p "请输入数字 [1-2]：" armnum
	case "$armnum" in
		1)
		roslaunch spark_carry_object hsv_detection.launch camera_type_tel:=${CAMERATYPE} color:=${calibration}
		;;
		2)
		roslaunch spark_carry_object hsv_detection.launch camera_type_tel:=${CAMERATYPE} color:=${color_block}
		;;
		*)
		echo -e "${Error} 错误，请填入正确的数字"
		;;
	esac

}



#让SPARK夺宝奇兵比赛用示例程序
spark_carry_game(){
	echo -e "${Info}" 
	echo -e "${Info}夺宝奇兵比赛用示例程序" 
	ROSVER=`/usr/bin/rosversion -d`
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash
	echo -e "${Info}请选择比赛方式：
	  ${Green_font_prefix}1.${Font_color_suffix} 手动模式
	  ${Green_font_prefix}2.${Font_color_suffix} 自动模式
	  ${Green_font_prefix}3.${Font_color_suffix} 退出请输入：Ctrl + c" 
	echo && stty erase ^? && read -p "请输入数字 [1-2]：" armnum
	case "$armnum" in
		1)
		roslaunch move2grasp teleop2grasp.launch camera_type_tel:=${CAMERATYPE}  lidar_type_tel:=${LIDARTYPE}
		;;
		2)
		roslaunch move2grasp move2grasp.launch camera_type_tel:=${CAMERATYPE}  lidar_type_tel:=${LIDARTYPE}
		;;
		*)
		echo -e "${Error} 错误，请填入正确的数字"
		;;
	esac	
}


clear_user_data(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash

	echo -e "${Info}" 
	echo -e "${Info}    清除用户数据"
	echo -e "${Info}将会删掉地图数据，ROS的log记录"

	echo && stty erase ^? && read -p "请选择是否继续y/n：" choose

	if [[ "${choose}" == "y" ]]; then
                rm -fr ${PROJECTPATH}/src/spark_app/spark_slam/scripts/test_map*
		rm -fr ~/.ros/*
		rosclean purge -y
		echo -e "${Info}    已清除用户数据"
	else
		return
	fi

}


qrcode_transfer_files(){
	wlp1s_ip=`/sbin/ifconfig wlp1s0|grep inet|awk '{print $2}'|awk -F: '{print $2}'`
	wlp2s_ip=`/sbin/ifconfig wlp2s0|grep inet|awk '{print $2}'|awk -F: '{print $2}'`
	wlan_ip=`/sbin/ifconfig wlan0|grep inet|awk '{print $2}'|awk -F: '{print $2}'`
	enp3s_ip=`/sbin/ifconfig enp3s0|grep inet|awk '{print $2}'|awk -F: '{print $2}'`
	eth_ip=`/sbin/ifconfig eth0|grep inet|awk '{print $2}'|awk -F: '{print $2}'`
	wlp0s_ip=`/sbin/ifconfig wlp0s20f3|grep inet|awk '{print $2}'|awk -F: '{print $2}'`
	if [ $wlp1s_ip ]; then
		echo -e "${Info}使用无线网络wlp1s0" 
	  	net_interface="wlp1s0"
	elif [ $wlp2s_ip ]; then
		echo -e "${Info}使用无线网络wlp2s0" 
	  	net_interface="wlp2s0"
	elif [ $wlan_ip ]; then
		echo -e "${Info}使用无线网络wlan0" 
	  	net_interface="wlan0"
	elif [ $enp3s_ip ]; then
		echo -e "${Info}使用无线网络enp3s0" 
		net_interface="enp3s0"
	elif [ $eth_ip ]; then
		echo -e "${Info}使用有线网络eth0" 
		net_interface="eth0"
	elif [ $wlp0s_ip ]; then
		echo -e "${Info}使用有线网络wlp0s20f3" 
		net_interface="wlp0s20f3"
	fi

	echo -e "${Info}" 
	echo -e "${Info}通过局域网收发文件" 
	echo -e "${Info}" 
	echo -e "${Info}请选择：
	  ${Green_font_prefix}1.${Font_color_suffix} 发送文件（文件名，带上文件绝对路径）
	  ${Green_font_prefix}2.${Font_color_suffix} 接收文件（默认存放在~/Downloads路径中）
	  ${Green_font_prefix}3.${Font_color_suffix} 退出请输入：Ctrl + c" 
	echo && stty erase ^? && read -p "请输入数字 [1-2]：" cnum
	case "$cnum" in
		1)
		echo -e "${Info}请输入文件名，带上文件绝对路径，如 /home/${USER}/a.jpg：
		 退出请输入：Ctrl + c" 
		echo && stty erase ^? && read -p "请输入要发送的文件：" s_file
		if [ -f "$s_file" ]; then
			echo -e "${Info}本机即将发送文件：${Green_font_prefix}"$s_file"${Font_color_suffix}，请接收端扫码或者直接输入下面的网址接收文件"
		else 
			echo -e "${Info}请输入带绝对路径的文件名"
			exit
		fi
		
		qrcp send  -i $net_interface $s_file
		;;
		2)
		echo -e "${Info}请输入接收到的文件存放的路径，默认为 /home/${USER}/Downloads：
		退出请输入：Ctrl + c" 
		echo && stty erase ^? && read -p "请输入文件存放的文件夹路径：" s_file
		if [ -d "$s_file" ]; then
			echo ""
		else 
			echo -e "${Info}${Red_font_prefix}文件夹不存在，将存放在默认文件夹/home/${USER}/Downloads中${Font_color_suffix}"
			s_file="/home/${USER}/Downloads"
		fi
		echo -e "${Info}接收的文件将存放在：${Green_font_prefix}"$s_file"${Font_color_suffix}，目录下，请发送端扫码或者直接输入下面的网址选择文件发送"
		qrcp  -i $net_interface receive --output=$s_file
		;;
		*)
		echo -e "${Error} 错误，退出"
		;;
	esac
}

coming_soon(){
	echo -e "${Tip} coming_soon!" 
}


#printf
menu_status(){
	echo -e "${Tip} 当前系统版本 ${OSDescription} !" 
	if [ -f "/usr/bin/rosversion" ]; then
		ROSVER=`/usr/bin/rosversion -d`
		if [ $ROSVER ]; then
			echo -e "${Tip} 当前ROS版本 ${ROSVER} !"
			return
		fi 
	fi
	echo -e "${Error} 未检测到ROS版本，请先安装ROS！可以选择102直接安装。" 
}

tell_us(){
	echo -e ""
	echo -e "${Tip} ------------分隔线--------------" 
	echo -e "${Tip} 网址：www.nxrobo.com" 
	echo -e "${Tip} SPARK技术讨论与反馈QQ群：一群 8346256(已满)；二群 6646169" 
	echo -e "${Tip} ------------分隔线--------------"
	echo -e ""
}

check_camera_game(){
  if [[ "${GAME_ENABLE}" == "yes" ]]; then 
	  check_camera_no_print
	  if [[ "${CAMERATYPE}" == "d435" ]]; then
		aa='a'
	  else
		echo -e "
  ${Green_font_prefix} 13.${Font_color_suffix} hsv值自动查找"
	  fi
		echo -e "
  ${Green_font_prefix} 20.${Font_color_suffix} 竞赛示例程序
	  
————————————"
  fi
}
check_sys
echo -e "  SPARK 一键安装管理脚本 ${Red_font_prefix}[v${sh_ver}]${Font_color_suffix}
  ---- J.xiao | www.nxrobo.com ----

  请根据右侧的功能说明选择相应的序号。
  注意：101～103为相关环境的安装与设置，如果已执行过，不要再重复执行。
  
  ${Green_font_prefix}  0.${Font_color_suffix} 单独编译SPARK
————————————
  ${Green_font_prefix}  1.${Font_color_suffix} 让机器人动起来
  ${Green_font_prefix}  2.${Font_color_suffix} 远程（手机APP）控制SPARK
  ${Green_font_prefix}  3.${Font_color_suffix} 让SPARK跟着你走
  ${Green_font_prefix}  4.${Font_color_suffix} 让SPARK使用激光雷达绘制地图
  ${Green_font_prefix}  5.${Font_color_suffix} 让SPARK使用深度摄像头绘制地图
  ${Green_font_prefix}  6.${Font_color_suffix} 让SPARK使用激光雷达进行导航
  ${Green_font_prefix}  7.${Font_color_suffix} 让SPARK使用深度摄像头进行导航
  ${Green_font_prefix}  8.${Font_color_suffix} 机械臂与摄像头标定
  ${Green_font_prefix}  9.${Font_color_suffix} 让SPARK通过机械臂进行视觉抓取
  ${Green_font_prefix} 10.${Font_color_suffix} 深度学习进行物品检测
  ${Green_font_prefix} 11.${Font_color_suffix} 语音控制SPARK移动
  ${Green_font_prefix} 12.${Font_color_suffix} 给摄像头做标定
———————————— "
check_camera_game
echo -e "
  ${Green_font_prefix}100.${Font_color_suffix} 问题反馈
  ${Green_font_prefix}101.${Font_color_suffix} 完整安装
  ${Green_font_prefix}102.${Font_color_suffix} 单独安装ROS环境
  ${Green_font_prefix}103.${Font_color_suffix} 单独安装SPARK依赖
  ${Green_font_prefix}104.${Font_color_suffix} 文件传输
  ${Green_font_prefix}105.${Font_color_suffix} 清除用户数据
 "
menu_status
check_dev
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
	spark_DeepLearn
	;;
	11)
	voice_nav
	;;
	12)
	calibrate_camera
	;;
	13)
	spark_hsv_detection
	;;
	20)
	spark_carry_game
	;;
	100)
	tell_us
	;;
	101)
	install_all
	;;
	102)
	check_install_ros_full
	;;
	103)
	install_spark_require
	;;
	104)
	qrcode_transfer_files
	;;
	105)
	clear_user_data
	;;	
	99)
	spark_dock
	;;
	98)
	spark_test_mode
	;;
	*)
	echo -e "${Error} 请输入正确的数字 "
	;;
esac
