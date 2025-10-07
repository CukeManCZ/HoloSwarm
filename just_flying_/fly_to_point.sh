#!/bin/sh
# fly_to_point.sh
# Publish a ReferenceStamped to /fly_to_waypoint once

ros2 topic pub --once /fly_to_waypoint mrs_msgs/msg/ReferenceStamped \
'header: 
  stamp: 
    sec: 0
    nanosec: 0
  frame_id: "" 
reference: 
  position: {x: 10.0, y: 10.0, z: 2.0}
  heading: 0.0'
