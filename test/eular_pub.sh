#!bin/bash
ros2 topic pub /drive/odom/eular geometry_msgs/msg/PoseStamped "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
pose:
  position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 270.0
    z: 0.0
    w: 1.0" 

