# Spark

This repository contains the ROS wrapper of Sparks's driver plus various ROS applications.This is a meta-package.

## Table of Contents

* [Update Log](#update-log)
* [Packages Overview](#packages-overview)
* [Usage](#usage)
* [Mirror](#mirror)
* [Routine](#routine)
  * [Spark-Follower](#spark-follower)
  * [Spark-SLAM Mapping](#spark-slam-mapping)
  * [Spark-Navigation](#spark-navigation)
  * [RTABMap-DeepCamera-Mapping](#rtabmap-deepcamera-mapping)
  * [Spark-RTABMap-Mapping-Navigation](#spark-rtabmap-mapping-navigation)
* [Routine (Chinese Version)](#routine-cn)
  * [Spark-跟随演示](#spark-跟随演示)
  * [Spark-SLAM地图构建](#spark-slam地图构建)
  * [Spark-自动导航](#spark-自动导航)
  * [RTABMap深度相机手持建图](#rtabmap-深度相机手持建图)
  * [Spark-RTABMap建图与导航](#spark-rtabmap建图与导航)

## Update Log

* Raise the stack so that the lidar can be added on it.
* Update the pre install packages so that the navigation and gmapping can be run.

## Packages Overview

* ***src*** : spark driver including base driver, camera driver, robot description, teleop package, and follow person package and so on.
* ***tools*** : it contains the 3rd part openni2 driver which camera driver uses.
* ***doc*** : it shows that how to compile and use this meta-package.

## Usage

### Prequirement

* System:	Ubuntu 14.04+
* ROS Version:	Indigo(Desktop-Full Install) 

### Compile

Build this compile with the following steps:
```yaml
git clone https://github.com/NXROBO/spark.git

#install
cd spark
./onekey.sh
```
If everything goes fine, test the follow-person example as follow:
```yaml
./install/follow_run.sh
```

# Mirror

We also provide a downloadable mirror whose all environments have been configured.
*  Download address: [spark_mirror](http://pan.baidu.com/s/1i4ZlH4p)



# Routine 



## Spark-Follower

### Introduction
* Spark will follow the object in front of itself and keep a certain distance. 

### Procedure
* Ensure spark base or camera are well connected with laptop.
* Configure the workspace 
```yaml
cd ~/spark_ws 
source devel/setup.bash
```

* Host computer, launch follower file
```yaml
roslaunch spark_follower bringup.launch
```

* Start following 
```yaml
Move Spark to open place, stand in front of spark. Spark will take corresponding action when you move back and forth or move left and right. 

You should control the moving speed and ensure that there doesn't exist too many objects around you. Otherwise, the following effect will not be guaranteed. 
```



## Spark-SLAM-Mapping

### Introduction

* Introduce the how to realize Gmapping, Hector, Karto, and Frontier Exploration algorithm on Spark for map construction
* Different algorithms apply to different situation, users can select appropriate method depending on actual tasks. 

### Procedure

* Default parameters are provided by ROS official website, which can be adapted for various requirements. 
* Ensure spark base or camera are well connected with laptop.
* Configure the workspace 

```yaml
cd ~/spark_ws 
source devel/setup.bash
```

* Host computer, launch SLAM file
```yaml
Based on 2d Lidar:
roslaunch spark_slam 2d_slam.launch slam_methods:=gmapping
No Rviz:
roslaunch spark_slam 2d_slam_norviz.launch slam_methods:=gmapping

Based on camera:
roslaunch spark_slam depth_slam.launch slam_methods:=gmapping 
No Rviz: 
roslaunch spark_slam depth_slam_norviz.launch slam=methods:=gmapping 
```

* Host computer, New terminal, launch keyboard control 
```yaml
rosrun spark_teleop spark_teleop_node 0.25 0.5 
  0.25 is linear velocity，0.5 is angular velocity，Spark can be controlled by ‘WASD’. 
```

* Host computer, New terminal, launch map saver 
```yaml
rosrun map_server map_saver -f ~/my_map
```
         
### Addition


```yaml
Note: Spark supports various SLAM methods

1. Spark supports Gmapping，Hector，Karto and Frontier Exploration.
2. Users can change slam_methods:=xxxx to choose methods, default is gmapping 
3. The value of 'slam_methods' includes gmapping, hector, karto, frontier_exploration
4. For example，if you want to use Karto Slam，select following instruction
   roslaunch spark_slam 2d_slam.launch slam_methods:=karto
```
         

```yaml
Note: Install corresponding package or download source code 

For Gmapping: 
Gmapping has been installed in install.sh

For Hector Mapping: 
sudo apt-get install ros-indigo-hector-mapping

For Frontier Exploration:
sudo apt-get install ros-indigo-frontier-exploration ros-indigo-navigation-stage 

For Karto Mapping:
sudo apt-get install ros-indigo-slam-karto
```

## Spark-Navigation

### Introduction 
* Given the map about surrounding environment, Spark can realize the automatic navigation and avoid static or active obstacles. 

### Procedure 
* Ensure spark base or camera are well connected with laptop.
* Configure the workspace 
```yaml
cd ~/spark_ws 
source devel/setup.bash
```

* Host computer, launch navigation file
```yaml
Based on 2d Lidar:
roslaunch spark_navigation amcl_demo_lidar.launch map_file:=home/username/my_map.yaml  (username is the name of host computer, map_file demonstrates the position of the map your construct and the name of your yaml file. )

Based on camera: 
roslaunch spark_navigation amcl_demo.launch map_file:=home/username/my_map.yaml 

"odom received!" indicates that the navigation initializes successfully.
```

* In Rviz, use ‘2D Pose Estimate’ to estimate the rough position of robot, and hold the left mouse to determine the robot’s orientation. 

* After pose estimation，use ‘2D Nav Goal’ to specify the goal and final orientation of robot. 


## RTABMap-DeepCamera-Mapping
### Introduction 
* RTAB-Map: Real Time Appearance-Based Mapping
* RTAB-Map is a RGB-D SLAM method based on global close-loop detection with real-time constraints.
* The method can generate 3D point cloud about surroundings. 

### Procedure 
* Ensure spark base or camera are well connected with laptop.
* Configure the workspace 
```yaml
cd ~/spark_ws 
source devel/setup.bash
```

* Host computer, launch depth camera
```yaml
roslaunch spark_rtabmap camera.launch
```
* Host computer, new terminal, launch mapping 
```yaml
roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start"

P.S. "--delete_db_on_start" is used to clear the old database.
```
* Move the camera slowly and stably to construct the map 

### Addition 
```yaml
Note: Install rtabmap package in advance
sudo apt-get install ros-indigo-rtabmap-ros
```


## Spark-RTABMap-mapping-navigation
### Introduction
* Use rtabmap-ros package to construct map and navigate on Spark
* The method can generate 3D point cloud and 2D occupied grid map about surroundings.


### Procedure
* Ensure spark base or camera are well connected with laptop.
* Configure the workspace 
```yaml
cd ~/spark_ws 
source devel/setup.bash
```

*  Host computer，launch spark and rtabmap related node
```yaml
roslaunch spark_rtabmap rtab_demo.launch
```
* Host, new terminal, launch mapping 
```yaml
roslaunch spark_rtabmap mapping.launch 

P.S. If there exists an error indicates that rtabmap node cannot be found in rtabmap_ros
source /opt/ros/indigo/setup.bash
```

* Host, new terminal, launch keyboard control 
```yaml
rosrun spark_teleop spark_teleop_node 0.25 0.5 
  0.25 is linear velocity，0.5 is angular velocity，Spark can be controlled by ‘WASD’. 
```
* Host, new terminal, launch localization and navigation
```yaml
roslaunch spark_rtabmap mapping.launch localization:=true
```
* In Rviz, use ‘2D Pose Estimate’ to estimate the rough position of robot, and hold the left mouse to determine the robot’s orientation. 
* After pose estimation，use ‘2D Nav Goal’ to specify the goal and final orientation of robot. 

## Addition 

```yaml
Note: Install rtabmap package in advance:
sudo apt-get install ros-indigo-rtabmap-ros

map data is saved in ~/.ros/rtabmap.db
```


# Routine-CN

## Spark-跟随演示

### 说明
* Spark的跟随演示会以它面前的一个目标为中心并保持一定的距离，如果太近即主动后退到适当距离，如果太远就主动跟上。

### 步骤 
* 确保Spark底盘、摄像头与主机连接良好
* 进行工作空间的配置
```yaml
cd ~/spark_ws  
source devel/setup.bash
```
*  主机，启动跟随文件
```yaml
roslaunch spark_follower bringup.launch 
```

* 开始跟随
```yaml
把Spark搬到开阔的地方，站在Spark前面，前后移动，Spark会根据摄像头获得信息进行相应的移动；左右移动时，Spark也会进行旋转，但左右移动速度不应过快。不要有太多物体在人的周围，Spark跟随的物体可能会改变。
```


## Spark-SLAM地图构建

### 说明
* 介绍Spark如何通过Gmapping， Hector SLAM， Karto SLAM以及Frontier Exploration等方法实现地图构建 
* 不同的建图方法可适用于不同的场景中，用户可根据自己的需求选择

### 步骤：
* 默认参数由ros官方的package提供，可自行修改以适应不同的情况
* 确保Spark底盘、Lidar或摄像头与主机连接良好
* 进行工作空间的配置
```yaml
cd ~/spark_ws 
source devel/setup.bash
```

* 主机，启动SLAM建图
```yaml 
基于2d lidar的建图:
roslaunch spark_slam 2d_slam.launch slam_methods:=gmapping
不启动Rviz:
roslaunch spark_slam 2d_slam_norviz.launch slam_methods:=gmapping

基于camera的建图:
roslaunch spark_slam depth_slam.launch slam_methods:=gmapping 	
不启动Rviz: 
roslaunch spark_slam depth_slam_norviz.launch slam=methods:=gmapping 
```

* 主机，新终端，启动键盘控制
```yaml
rosrun spark_teleop spark_teleop_node 0.25 0.5 
  0.25为线速度，0.5为角速度，用户可通过WASD控制Spark移动
```

* 主机，新终端，保存地图
```yaml
rosrun map_server map_saver -f ~/my_map
```
 	
### 补充

``` yaml
Note: Spark支持多种SLAM建图方法

1. Spark支持Gmapping，Hector，Karto以及Frontier Exploration方法
2. 用户可以通过更改slam_methods:=xxxx选择不同的方法，默认方法为gmapping
3. slam_methods包括gmapping, hector, karto, frontier_exploration
4. 举个例子，如果想使用Karto Slam，可以用如下指令
   roslaunch spark_slam 2d_slam.launch slam_methods:=karto
```
         

```yaml
Note: 安装对应的package或下载对应源码

For Gmapping: 
Gmapping包在install.sh中已经安装

For Hector Mapping: 
sudo apt-get install ros-indigo-hector-mapping

For Frontier Exploration:
sudo apt-get install ros-indigo-frontier-exploration ros-indigo-navigation-stage 

For Karto Mapping:
sudo apt-get install ros-indigo-slam-karto
```



## Spark-自动导航

### 说明
* 在构建好周围环境地图的前提下，Spark可以在地图范围内实现自动导航并且能够规避动态和静态的障碍物
### 步骤
* 确保Spark底盘、摄像头或雷达与主机连接良好
* 进行工作空间的配置
```yaml
cd ~/spark_ws 
source devel/setup.bash
```

* 主机，启动导航文件
```yaml
基于2d lidar:
roslaunch spark_navigation amcl_demo_lidar.launch map_file:=home/username/my_map.yaml  (username为主机用户名, map_file后面为地图的yaml文件所在位置, 可根据地图保存的地址进行更改)

基于camera: 
roslaunch spark_navigation amcl_demo.launch map_file:=home/username/my_map.yaml 

如果你看到 odom received! 说明已经正常运行。
```

* 在Rviz中选择“2D Pose Estimate”估计Spark大概的位置。按住鼠标左键估计确定Spark大概朝向。

* 设置好估计的姿态，选择“2D Nav Goal”，点击你想让Spark去的地方以及朝向。


## RTABMap-深度相机手持建图

### 说明
* RTAB-Map：Real-Time Appearance-Based Mapping
* RTAB-Map是基于具有实时约束的全局闭环检测的RGB-D SLAM方法
* 可以生成环境的3D点云

### 步骤
* 确保Spark底盘、摄像头与主机连接良好
* 进行工作空间的配置
* cd ~/spark_ws 
```yaml
source devel/setup.bash
```
* 主机，启动深度相机
```yaml
roslaunch spark_rtabmap camera.launch
```
* 主机，新终端，启动建图模式
```yaml
roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start"

P.S. "--delete_db_on_start"用于清除旧数据库
```
* 缓慢移动camera进行建图 

### 补充
```yaml
Note: 在实验前要安装rtabmap package 
sudo apt-get install ros-indigo-rtabmap-ros
```

## Spark-RTABMap建图与导航

### 说明
* 利用rtabmap-ros包在spark上实现建图与导航
* 可生成3D点云以及2D占据栅格地图

### 步骤
* 确保Spark底盘、摄像头与主机连接良好
* 进行工作空间的配置
```yaml
cd ~/spark_ws 
source devel/setup.bash
```

*  主机，启动spark以及rtabmap相关节点
```yaml
roslaunch spark_rtabmap rtab_demo.launch
```

* 主机，新终端，启动建图
```yaml
roslaunch spark_rtabmap mapping.launch 

P.S. 这一步若提示在rtabmap_ros找不到rtabmap
source /opt/ros/indigo/setup.bash
```

* 主机，新终端，启动键盘控制
```yaml
rosrun spark_teleop spark_teleop_node 0.25 0.5 
  0.25为线速度，0.5为角速度，用户可通过WASD控制Spark移动
```
* 主机，新终端，启动定位导航模式
```yaml
roslaunch spark_rtabmap mapping.launch localization:=true
```
* 在Rviz中选择“2D Pose Estimate”估计Spark大概的位置。按住鼠标左键估计确定Spark大概朝向。
* 设置好估计的姿态，选择“2D Nav Goal”，点击你想让Spark去的地方以及朝向。

### 补充

```yaml
Note: 在实验前要安装rtabmap package 
sudo apt-get install ros-indigo-rtabmap-ros

地图数据保存在 ~/.ros/rtabmap.db中
```
