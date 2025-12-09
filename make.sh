#!/bin/bash

# colcon build --symlink-install --executor parallel --parallel $(nproc)
colcon build --executor parallel --parallel $(nproc)