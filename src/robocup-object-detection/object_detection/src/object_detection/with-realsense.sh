#!/bin/bash
args=$@
SCRIPT_DIR=$(cd $(dirname "${BASH_SOURCE[0]}") && pwd)
source $SCRIPT_DIR/../../../../../openvino_2022/setupvars.sh
sleep 5
roslaunch object_detection detect_objects.launch $args
pkill -P $$
