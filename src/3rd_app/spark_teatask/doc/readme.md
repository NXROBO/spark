启动步骤
1.确定网络连接正常
2.确定麦克风及喇叭设置正确
3.建立好地图
4.启动端茶任务
roslaunch spark_fetchtea spark_fetchtea.launch
5.通过rviz pose estimate设定机器人定位初始点，然后遥控机器人知道粒子云收敛
6.指定厨房位置，通过启动的rviz，通过publish point来选取一个点，rviz上会显示kitchen文字在rviz里

对麦克风说话：去端茶
spark反应：到机器人指定的厨房位置

对麦克风说话：回家
spark反应：到机器人指定的机器人原点位置

