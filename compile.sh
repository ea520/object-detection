#!/bin/bash
args=$@
source /opt/intel/openvino_2022/setupvars.sh
catkin_make $args
