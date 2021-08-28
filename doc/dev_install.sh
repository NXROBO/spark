echo 'Spark driver is installing'

echo 'Setting udev rules'
BASEPATH=$(cd `dirname $0`; pwd)
sudo cp $BASEPATH/rules/3ilidar-usb-serial.rules /etc/udev/rules.d/
sudo cp $BASEPATH/rules/spark-usb-serial.rules /etc/udev/rules.d/
sudo cp $BASEPATH/rules/orbbec-usb.rules /etc/udev/rules.d/556-orbbec-usb.rules
sudo udevadm trigger

echo 'Installing required libs'
sudo apt-get update
sudo apt-get install ros-indigo-ecl ros-indigo-ecl-threads ros-indigo-rgbd-launch 
sudo apt-get install ros-indigo-image-common
sudo apt-get install ros-indigo-move-base-* 
sudo apt-get install ros-indigo-depthimage-to-laserscan ros-indigo-map-server ros-indigo-amcl ros-indigo-gmapping ros-indigo-navigation*
sudo apt-get install libasound2-dev mplayer
echo 'Spark driver is installed'

