#!/bin/bash

rm -rf build/ install/ log/

colcon build --packages-select ros2_utils ros2_interface
# colcon build --symlink-install --executor parallel --parallel $(nproc)
#colcon build --executor parallel --parallel $(nproc)
