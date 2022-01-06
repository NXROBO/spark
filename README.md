﻿﻿# NXROBO Spark
<img src="https://raw.githubusercontent.com/NXROBO/spark/master/doc/spark.jpg" width="300">



- windows相关产品了解更多，请使用微信扫描以下二维码：
  <img src="https://raw.githubusercontent.com/NXROBO/spark/master/doc/ewm.png" width="300">

  

## 列表 Table of Contents

* [功能包说明packages-overview](#功能包说明packages-overview)
* [使用usage](#使用usage)
* [视频展示Video](#视频展示Video)



## 功能包说明packages-overview

* ***src*** : Spark的源代码，包括底层配置，硬件驱动，和各个应用功能包等。
* ***doc*** : 软硬件依赖包。



## 使用usage

### 系统要求 Prequirement

* System:  Ubuntu 16.04+
* ROS Version:  kinetic (Desktop-Full Install) 

### 下载安装 Download and install

* 下载工作空间 Download the workspace:
```bash
git clone https://github.com/NXROBO/spark.git
```
* 安装依赖库 Install libraries and dependencies:
```bash
cd spark
./onekey.sh
```
* 根据提示选择103 Choose NO.103
```bash
103
```
### 编译运行 compile and run
```bash
catkin_make
```
* 如果编译一切正常，可根据提示运行相关例程。If everything goes fine, test the examples as follow:
```bash
./onekey.sh
```



## 参数说明 Arguments description

Launch file arguments description

### camera_type_tel

深度摄像头的型号，其对应值如下表。The model type of the depth camera, and the corresponding values can be seen below...

| Camera version       | CAMERATYPE value |
| -------------------- | ---------------- |
| Astra Pro            | "astrapro"       |
| Astra                | "astra"          |
| Intel RealSense D435 | "d435"           |



### depthtolaser

深度摄像头的深度信息topic，其对应值如下表。The depth information topic of the depth camera, and the corresponding values can be seen below...

| Camera version       | depthtolaser value             |
| -------------------- | ------------------------------ |
| Astra Pro            | "/camera/depth/image_rect_raw" |
| Astra                | "/camera/depth/image_raw"      |
| Intel RealSense D435 | "/camera/depth/image_rect_raw" |



### lidar_type_tel

激光雷达的型号，其对应值如下表。The model type of the lidar, and the corresponding values can be seen below...

| Lidar version    | LIDARTYPE value    |
| ---------------- | ------------------ |
| YDLIDAR G2       | "ydlidar_g2"       |
| 3iRobotics lidar | "3iroboticslidar2" |



### slam_methods_tel

SLAM的算法，其对应值如下表。The algorithm of the SLAM, and the corresponding values can be seen below...

| SLAM algorithm             | SLAMTYPE value         |
| -------------------------- | ---------------------- |
| gmapping                   | "gmapping"             |
| hectorSLAM                 | "hector"               |
| Frontier-based exploration | "frontier_exploration" |
| kartoSLAM                  | “karto”                |



## 视频展示Video

1.Spark跟随 Spark-Follower

<a href="https://www.youtube.com/embed/UrD2AEQ3VkI" target="_blank"><img src="http://img.youtube.com/vi/UrD2AEQ3VkI/0.jpg" 
alt="follow-person" width="240" height="180" border="10" /></a>

```bash
cd spark
source devel/setup.bash

roslaunch spark_follower bringup.launch camera_type_tel:=CAMERATYPE
```

2.Spark建图 Spark-SLAM-Mapping

<a href="https://www.youtube.com/embed/Yt9Sld-EX0s" target="_blank"><img src="http://img.youtube.com/vi/Yt9Sld-EX0s/0.jpg" 
alt="follow-person" width="240" height="180" border="10" /></a>

```bash
cd spark
source devel/setup.bash

roslaunch spark_slam 2d_slam_teleop.launch slam_methods_tel:=SLAMTYPE camera_type_tel:=CAMERATYPE lidar_type_tel:=LIDARTYPE
```

3.Spark导航 Spark-Navigation

<a href="https://www.youtube.com/embed/3RP11sZKfJg" target="_blank"><img src="http://img.youtube.com/vi/3RP11sZKfJg/0.jpg" 
alt="follow-person" width="240" height="180" border="10" /></a>

```bash
cd spark
source devel/setup.bash

roslaunch spark_navigation amcl_demo_lidar_rviz.launch camera_type_tel:=CAMERATYPE lidar_type_tel:=LIDARTYPE
```

4.Spark-RtabMap建图 Spark-RtabMap-Mapping

<a href="https://www.youtube.com/embed/K5wvlWb-2uQ" target="_blank"><img src="http://img.youtube.com/vi/K5wvlWb-2uQ/0.jpg" 
alt="follow-person" width="240" height="180" border="10" /></a>
```bash
cd spark
source devel/setup.bash

roslaunch spark_slam depth_slam_teleop.launch slam_methods_tel:=SLAMTYPE camera_type_tel:=CAMERATYPE depthtolaser:=depthtolaser
```

5.Spark机械臂视觉抓取 Spark-Carry_Object

<a href="https://www.youtube.com/embed/aNPy6GYcdu0" target="_blank"><img src="http://img.youtube.com/vi/aNPy6GYcdu0/0.jpg" 
alt="follow-person" width="240" height="180" border="10" /></a>

```bash
cd spark
source devel/setup.bash

roslaunch spark_carry_object spark_carry_object_only_cv3.launch camera_type_tel:=CAMERATYPE lidar_type_tel:=LIDARTYPE
```
