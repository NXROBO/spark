#!/bin/bash
export DISPLAY=:0.0
source /opt/ros/indigo/setup.bash
export ROS_MASTER_URI=http://localhost:11311
source ./install/setup.bash

roslaunch spark_follower bringup.launch
