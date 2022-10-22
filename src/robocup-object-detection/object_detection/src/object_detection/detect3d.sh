#!/bin/bash
args=$@
source /opt/intel/openvino_2022/setupvars.sh
roslaunch object_detection detect_objects.launch $args
