#! /bin/bash

source ../../../devel/setup.sh 
rosparam set cv_camera/device_id 1 
roslaunch qrfinder.launch 
