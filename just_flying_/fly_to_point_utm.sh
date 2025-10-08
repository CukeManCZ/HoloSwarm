#!/bin/sh
# fly_to_point.sh
# Publish a ReferenceStamped to /fly_to_waypoint once

ros2 topic pub --once /fly_to_waypoint mrs_msgs/msg/ReferenceStamped \
'header: 
  stamp: 
    sec: 0
    nanosec: 0
  frame_id: "/uav1/utm_origin" 
reference: 
  position: {x: 446372.200, y: 5467988.960, z: 342.834}
  heading: 0'

