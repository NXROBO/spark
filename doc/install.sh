echo 'Spark driver is installing'

echo 'Setting udev rules'
BASEPATH=$(cd `dirname $0`; pwd)
sudo cp $BASEPATH/rules/spark-usb-serial.rules /etc/udev/rules.d/
sudo cp $BASEPATH/rules/orbbec-usb.rules /etc/udev/rules.d/556-orbbec-usb.rules
sudo udevadm trigger

echo 'Installing required libs'
sudo apt-get update
sudo apt-get install ros-kinetic-ecl ros-indigo-ecl-threads ros-kinetic-rgbd-launch 
sudo apt-get install ros-kinetic-image-common
sudo apt-get install ros-kinetic-move-base-* 
sudo apt-get install ros-kineitc-depthimage-to-laserscan ros-kientic-map-server ros-kinetic-amcl
sudo apt-get install libasound2-dev mplayer
echo 'Spark driver is installed'

