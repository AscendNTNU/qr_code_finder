#! /bin/bash

source ../../../devel/setup.sh 
rosparam set cv_camera/device_id 0 
roslaunch qrfinder.launch 
