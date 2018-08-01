# SwiftAndProForROS
This is the swift and swiftpro ROS package designed by Roger Cui(roger@ufactory.cc) and David Long (xiaokun.long@ufactory.cc).
These packages support Moveit!, RViz and serial communication with swift and swiftpro.

## 1. Download and install
Download ros packages for uarm swift pro
```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/uArm-Developer/RosForSwiftAndSwiftPro.git
```
then manually copy package folders *swiftpro* *swift_moveit_config* and *pro_moveit_config* into a catkin_ws/src.

Install ros serial package
```bash
$ sudo apt-get install ros-kinetic-serial
```

Compile
```bash
$ catkin_make
```

## 2. Set up enviroment
Source all setup.bash files to set up your enviroment.
```bash
# System configure ROS environment variables automatically every time you open a ternimal
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 3. Simulation
Connect your swift/swiftpro to computer and get USB permission to access uArm
```bash
$ sudo chmod 666 /dev/ttyACM0
```

### 3.1 Display mode: Get data from swiftpro
Get data from serial and simulate swiftpro in RViz.
```bash
roslaunch swiftpro pro_display.launch
```
right now, you can drag your swiftpro and it will simulate in Rviz.

### 3.2 Control Mode: Send data to swiftpro
Connect swiftpro, send data though serial.
```bash
roslaunch swiftpro pro_control.launch
```
Open another ternimal to get joint angles from Moveit!.
```bash
roslaunch pro_moveit_config demo.launch
```
right now, you can do trajectory planning or grasping in moveIt!.

### 3.3 Display mode: Get data from swift
Get data from serial and simulate swift in RViz.
```bash
roslaunch swiftpro swift_display.launch
```
right now, you can drag your swift and it will simulate in Rviz.

### 3.4 Control Mode: Send data to swift
Connect swift, send data though serial.
```bash
roslaunch swiftpro swift_control.launch
```
Open another ternimal to get joint angles from Moveit!.
```bash
roslaunch swift_moveit_config demo.launch
```
right now, you can do trajectory planning or grasping in moveIt!.

### 3.5 About nodes and topics
<img src="http://obmqyor62.bkt.clouddn.com/swift.jpg" width = "780" height = "350" />

### 3.6 About message
SwiftproState.msg: includes all data about swiftpro
```bash
float64 motor_angle1
float64 motor_angle2
float64 motor_angle3
float64 motor_angle4
float64 x
float64 y
float64 z
uint8   pump
uint8   swiftpro_status
uint8   gripper
```
position.msg: includes x, y, z information(mm)
```bash
float64 x
float64 y
float64 z
```
angle4th.msg: 4th motor angle(degree)
```bash
float64 angle4th
```
status.msg: work if 1; otherwise 0
```bash
uint8 status
```
