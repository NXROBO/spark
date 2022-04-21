echo '安装EAI G6雷达'
sudo cp ~/spark/doc/lidar_ydlidar_g6.txt /opt/lidar.txt
sudo chmod 755 ~/spark/src/spark_driver/lidar/ydlidar_g6/startup/initenv.sh 
sudo ~/spark/src/spark_driver/lidar/ydlidar_g6/startup/initenv.sh
echo '安装完成'
echo '请重新拔插雷达的USB接口'


