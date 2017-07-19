1.机械臂抓取物体
a.远程登录
ssh spark@ip
b.启动抓取物体功能模块
roslaunch spark_carry_object spark_carry_object_only.launch
c.开始抓取
rosservice call /s_carry_object "type: 1
param: ''" 

2.随行功能
a.进入目录
cd ~/sparkws/install
b.运行随行功能
./follow_run.sh
c.将电脑放在spark卡槽上


