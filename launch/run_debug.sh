#! /bin/bash

source ../../../devel/setup.sh 
#rosparam set cv_camera/device_id 1 
#rosparam set cv_camera/image_width 1920
#rosparam set cv_camera/image_height 1080
#roslaunch camera_publisher1.launch
roslaunch qrfinder.launch 
