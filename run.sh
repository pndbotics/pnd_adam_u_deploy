#!/bin/bash

source /opt/ros/humble/setup.bash
source "$(dirname "$0")/install/setup.bash"
export ROS_LOCALHOST_ONLY=1

ros2 run pnd_adam_u_deploy publisher_node


