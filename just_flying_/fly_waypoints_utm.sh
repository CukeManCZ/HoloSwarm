#!/bin/sh
# fly_waypoints.sh
# Publish a ReferenceArray to /fly_through_waypoints once
# Converted from world_origin (relative) to absolute UTM

ros2 topic pub --once /fly_through_waypoints mrs_msgs/msg/ReferenceArray \
'header: 
  stamp: 
    sec: 0
    nanosec: 0
  frame_id: "/uav8/utm_origin" 
array: 
  - position: {x: 446372.200, y: 5467988.960, z: 342.834}
    heading: 0.0
  - position: {x: 446377.200, y: 5467983.960, z: 342.834}
    heading: 0.0
  - position: {x: 446377.200, y: 5467993.960, z: 342.834}
    heading: 0.0
  - position: {x: 446367.200, y: 5467993.960, z: 342.834}
    heading: 0.0
  - position: {x: 446367.200, y: 5467983.960, z: 342.834}
    heading: 0.0
  - position: {x: 446377.200, y: 5467983.960, z: 342.834}
    heading: 0.0
  - position: {x: 446372.200, y: 5467988.960, z: 342.834}
    heading: 0.0'

