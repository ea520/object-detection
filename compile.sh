#!/bin/bash
args=$@
source openvino_2022/setupvars.sh
catkin_make $args
