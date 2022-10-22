#!/bin/bash
args=$@
source /opt/intel/openvino_2022/setupvars.sh
rosrun object_detection .detect2d $args
