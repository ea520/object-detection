#!/bin/bash
filepath=$1
args=${@:2}
SCRIPT_DIR=$(cd $(dirname "${BASH_SOURCE[0]}") && pwd)
source $SCRIPT_DIR/../../../../../openvino_2022/setupvars.sh
roslaunch object_detection detect_objects.launch $args&
rosbag play -r 1 $filepath > /dev/null
pkill -P $$