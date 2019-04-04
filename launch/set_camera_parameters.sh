#! /bin/bash

v4l2-ctl -d /dev/video1 --set-ctrl exposure_auto=1
v4l2-ctl -d /dev/video1 --set-ctrl exposure_absolute=80

