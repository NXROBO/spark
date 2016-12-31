sudo cp ./libOpenNI2.so.0 /usr/lib/
sudo cp ./libOpenNI2.jni.so.0 /usr/lib/

sudo rm -rf /usr/lib/OpenNI2/Drivers/*
sudo cp ./OpenNI2/Drivers/* /usr/lib/OpenNI2/Drivers
