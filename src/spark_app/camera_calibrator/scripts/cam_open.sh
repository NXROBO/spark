#!/bin/bash

cameraname=$1
gnome-terminal -x bash -c "roslaunch camera_driver_transfer ${cameraname}.launch; exec bash"


