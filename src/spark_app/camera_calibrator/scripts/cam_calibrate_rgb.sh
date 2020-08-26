#!/bin/bash

cameraname=$1

if [ "${cameraname}"== "astra" ]||[ "${cameraname}" == "astrapro" ]; then
	gnome-terminal -x bash -c "rosrun camera_calibration cameracalibrator.py --size 9x7 --square 0.020 image:=/camera/rgb/image_raw; exec bash"
elif [ "${cameraname}" == "d435" ]; then
	gnome-terminal -x bash -c "rosrun camera_calibration cameracalibrator.py --size 9x7 --square 0.020 image:=/camera/color/image_raw; exec bash"
fi



