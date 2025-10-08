#!/bin/bash
# fly_waypoints.sh
# Publishes a ReferenceArray with computed UTM offsets
# Works reliably with ROS2 topic pub


source uav_origin_offset.sh

# Default is 3m in Z
BASE_X=446372.200
BASE_Y=5467988.960
BASE_Z=342.834

# Compute offsets
X=$(printf "%.3f" "$(echo "$BASE_X + $X_OFFSET" | bc -l)")
Y=$(printf "%.3f" "$(echo "$BASE_Y + $Y_OFFSET" | bc -l)")
Z=$(printf "%.3f" "$(echo "$BASE_Z + $HEIGHT_OFFSET" | bc -l)")

# Precompute all waypoint positions
X1=$(printf "%.3f" "$(echo "$X + 0.0" | bc -l)")
Y1=$(printf "%.3f" "$(echo "$Y + 0.0" | bc -l)")
Z1=$Z

X2=$(printf "%.3f" "$(echo "$X + 5.0" | bc -l)")
Y2=$(printf "%.3f" "$(echo "$Y - 5.0" | bc -l)")
Z2=$Z

X3=$(printf "%.3f" "$(echo "$X + 5.0" | bc -l)")
Y3=$(printf "%.3f" "$(echo "$Y + 5.0" | bc -l)")
Z3=$Z

X4=$(printf "%.3f" "$(echo "$X - 5.0" | bc -l)")
Y4=$(printf "%.3f" "$(echo "$Y + 5.0" | bc -l)")
Z4=$Z

X5=$(printf "%.3f" "$(echo "$X - 5.0" | bc -l)")
Y5=$(printf "%.3f" "$(echo "$Y - 5.0" | bc -l)")
Z5=$Z

X6=$(printf "%.3f" "$(echo "$X + 5.0" | bc -l)")
Y6=$(printf "%.3f" "$(echo "$Y - 5.0" | bc -l)")
Z6=$Z

X7=$(printf "%.3f" "$(echo "$X + 0.0" | bc -l)")
Y7=$(printf "%.3f" "$(echo "$Y + 0.0" | bc -l)")
Z7=$Z

# Debug print (optional)
echo "Publishing waypoints around ($X, $Y, $Z)"

# Publish the array
ros2 topic pub --once /fly_through_waypoints mrs_msgs/msg/ReferenceArray "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'uav1/utm_origin'
array:
  - position: {x: $X1, y: $Y1, z: $Z1}
    heading: 0.0
  - position: {x: $X2, y: $Y2, z: $Z2}
    heading: 0.0
  - position: {x: $X3, y: $Y3, z: $Z3}
    heading: 0.0
  - position: {x: $X4, y: $Y4, z: $Z4}
    heading: 0.0
  - position: {x: $X5, y: $Y5, z: $Z5}
    heading: 0.0
  - position: {x: $X6, y: $Y6, z: $Z6}
    heading: 0.0
  - position: {x: $X7, y: $Y7, z: $Z7}
    heading: 0.0"

