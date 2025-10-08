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
  - position: {x: 5.0, y: -5.0, z: 3.0}
    heading: 0.0
  - position: {x: 5.0, y: 5.0, z: 3.0}
    heading: 0.0
  - position: {x: -5.0, y: 5.0, z: 3.0}
    heading: 0.0
  - position: {x: -5.0, y: -5.0, z: 3.0}
    heading: 0.0
  - position: {x: 5.0, y: -5.0, z: 3.0}
    heading: 0.0
  - position: {x: -0.0, y: -0.0, z: 3.0}
    heading: 0.0'

