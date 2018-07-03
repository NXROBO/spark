echo 'Spark driver is installing'

echo 'Setting udev rules'
BASEPATH=$(cd `dirname $0`; pwd)
sudo cp $BASEPATH/rules/spark-usb-serial.rules /etc/udev/rules.d/
sudo cp $BASEPATH/rules/orbbec-usb.rules /etc/udev/rules.d/556-orbbec-usb.rules
sudo udevadm trigger

echo 'Installing required libs'
sudo apt-get update
sudo apt-get install ros-melodic-ecl ros-melodic-ecl-threads ros-melodic-rgbd-launch 
sudo apt-get install ros-melodic-image-common
sudo apt-get install ros-melodic-move-base-* 
sudo apt-get install ros-melodic-depthimage-to-laserscan ros-melodic-map-server ros-melodic-amcl ros-melodic-gmapping ros-melodic-navigation*
sudo apt-get install libasound2-dev mplayer
echo 'Spark driver is installed'

