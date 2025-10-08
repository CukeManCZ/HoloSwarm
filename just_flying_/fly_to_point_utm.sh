##!/bin/bash
# fly_to_point_utm.sh
# Publish a ReferenceStamped to /fly_to_waypoint once

source uav_origin_offset.sh

#Default is 3m in Z
BASE_X=446372.200
BASE_Y=5467988.960
BASE_Z=472.9

X=$(echo "$BASE_X + $X_OFFSET" | bc)
Y=$(echo "$BASE_Y + $Y_OFFSET" | bc)
Z=$(echo "$BASE_Z + $HEIGHT_OFFSET" | bc)

ros2 topic pub --once /fly_to_waypoint mrs_msgs/msg/ReferenceStamped \
"header: 
  stamp: 
    sec: 0
    nanosec: 0
  frame_id: "uav8/utm_origin" 
reference: 
  position: {x: $X, y: $Y, z: $Z}
  heading: 0"

