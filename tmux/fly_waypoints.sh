#!/bin/sh
# fly_waypoints.sh
# Publish a ReferenceArray to /fly_through_waypoints once

ros2 topic pub --once /fly_through_waypoints mrs_msgs/msg/ReferenceArray \
'header: 
  stamp: 
    sec: 0
    nanosec: 0
  frame_id: "" 
array: 
  - position: {x: 0.0, y: 0.0, z: 3.0}
    heading: 0.0
  - position: {x: 3.0, y: -2.0, z: 3.0}
    heading: 0.0
  - position: {x: -4.0, y: 1.5, z: 3.0}
    heading: 0.0
  - position: {x: 2.5, y: 4.0, z: 3.0}
    heading: 0.0
  - position: {x: -1.0, y: -3.5, z: 3.0}
    heading: 0.0'
