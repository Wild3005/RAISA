#!/bin/bash
set -e

TIMESTAMP=$(date +"%d_%m_%y_%H_%M_%S")
BAG_NAME="rosbag_${TIMESTAMP}"

TOPICS_CAM=(
  /reeman/pose
  /reeman/nav_status
  /reeman/point_cloud

  /dual_leg
  /ui/human/pose2d
  /ui/human/velocity
  /ui/human/mode

  /vision/person_position
  /vision/pose_detected
  /vision/person_orientation
  /vision/image_raw

)

TOPICS_UWB=(
  /reeman/pose
  /reeman/nav_status
  /reeman/point_cloud

  /dual_leg
  /ui/human/pose2d
  /ui/human/velocity
  /ui/human/mode

  /uwb_pose2d
)

echo "[INFO] Recording topics:"
printf ' - %s\n' "${TOPICS[@]}"
echo "[INFO] Output bag: $BAG_NAME"

# ROS1
# rosbag record "${TOPICS[@]}" -O "$BAG_NAME"

# ROS2 alternative
ros2 bag record "${TOPICS_UWB[@]}" -o "$BAG_NAME"
ros2 bag record "${TOPICS_CAM[@]}" -o "$BAG_NAME"
