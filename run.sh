#!/bin/bash

. install/setup.bash 
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=14
ros2 launch ros2_utils all.launch.py 

