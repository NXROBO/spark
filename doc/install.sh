echo 'Spark driver is installing'

echo 'Setting udev rules'
sudo cp ./rules/70-ttyusb.rules /etc/udev/rules.d/
sudo cp ./rules/orbbec-usb.rules /etc/udev/rules.d/556-orbbec-usb.rules

echo 'Copying libmsc.so to /usr/lib/'
sudo cp ../spark_tutorials/spark_iflytek/lib/x64/libmsc.so /usr/lib/

echo 'Installing required libs'
sudo apt-get install ros-indigo-ecl ros-indigo-ecl-threads ros-indigo-rgbd-launch 
sudo apt-get install ros-indigo-image-common
sudo apt-get install ros-indigo-move-base-* 
sudo apt-get install ros-indigo-depthimage-to-laserscan ros-indigo-map-server ros-indigo-amcl
sudo apt-get install libasound2-dev mplayer
echo 'Spark driver is installed'

