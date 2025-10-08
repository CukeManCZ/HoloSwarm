#!/bin/bash
# fly_to_point.sh
# Publish a ReferenceStamped to /fly_to_waypoint once

source uav_origin_offset.sh

BASE_X=0.0
BASE_Y=0.0
BASE_Z=3.0

X=$(echo "$BASE_X + $X_OFFSET" | bc)
Y=$(echo "$BASE_Y + $Y_OFFSET" | bc)
Z=$(echo "$BASE_Z + $HEIGHT_OFFSET" | bc)

ros2 topic pub --once /fly_to_waypoint mrs_msgs/msg/ReferenceStamped \
"header: 
  stamp: 
    sec: 0
    nanosec: 0
  frame_id: "uav1/world_origin" 
reference: 
  position: {x: $X, y: $Y, z: $Z}
  heading: 0.0"
