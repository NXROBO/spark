echo '安装EAI G2雷达'
sudo cp ~/spark/doc/lidar_ydlidar_g2.txt /opt/lidar.txt
sudo chmod 755 ~/spark/src/spark_driver/lidar/ydlidar_g2/startup/initenv.sh 
sudo ~/spark/src/spark_driver/lidar/ydlidar_g2/startup/initenv.sh
echo '安装完成'
echo '请重新拔插雷达的USB接口'


