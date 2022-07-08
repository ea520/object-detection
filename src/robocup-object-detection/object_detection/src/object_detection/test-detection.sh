#!/bin/bash
args=$@
SCRIPT_DIR=$(cd $(dirname "${BASH_SOURCE[0]}") && pwd)
source $SCRIPT_DIR/../../../../../openvino_2022/setupvars.sh
rosrun object_detection .detect2d $args