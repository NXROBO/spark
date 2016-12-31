sudo rm /usr/lib/libOpenNI2.so.0
sudo rm /usr/lib/libOpenNI2.jni.so.0
sudo rm /usr/lib/libOpenNI2.so
sudo rm /usr/lib/libOpenNI2.jni.so
sudo rm /usr/lib/OpenNI.ini

sudo cp ./libOpenNI2.so.0 /usr/lib/
sudo cp ./libOpenNI2.jni.so.0 /usr/lib/
sudo cp ./libOpenNI2.so /usr/lib/
sudo cp ./libOpenNI2.jni.so /usr/lib/
sudo cp ./OpenNI.ini /usr/lib/

sudo rm -rf /usr/lib/OpenNI2/Drivers/*
sudo cp ./OpenNI2/Drivers/* /usr/lib/OpenNI2/Drivers
