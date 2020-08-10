#!/bin/bash


gnome-terminal -x bash -c "rosrun camera_calibration cameracalibrator.py --size 9x7 --square 0.020 image:=/camera/rgb/image_raw; exec bash"

