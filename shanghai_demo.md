1.机械臂抓取物体
a.远程登录
ssh spark@192.168.1.101
b.启动抓取物体功能模块
cd sparkws
source devel/setup.bash
sudo chmod 666 /dev/ttyACM0
roslaunch spark_carry_object spark_carry_object_only.launch
c.开始抓取
cd ~/spark
source devel/setup.bash
// 一起粘贴
rosservice call /s_carry_object "type: 1
param: ''" 

2.随行功能
a.进入目录
cd ~/sparkws/install
b.运行随行功能
./follow_run.sh


3.建图功能
a.启动建图启动文件
roslaunch spark_navigation gmapping_demo_lidar.launch
b.启动机器人描述文件（在新的终端）
roslaunch spark_description spark_description.launch
c.启动机器人控制
roslaunch spark_teleop teleop.launch

4.导航功能
a.启动导航启动文件
roslaunch spark_navigation amcl_demo_lidar.launch
b.启动机器人描述文件（在新的终端）
roslaunch spark_description spark_description.launch
c.在rviz里面添加map话题 增加代价地图效果（costmap)


