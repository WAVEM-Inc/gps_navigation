# gps_navigation
auto-driving package with gps

### DEV
![FrameWork](https://img.shields.io/badge/ros2-humble-blue?style=for-the-badge&logo=ROS&logoColor=White)
![OS](https://img.shields.io/badge/ubuntu-22.04-red?style=for-the-badge&logo=Ubuntu&logoColor=%23FFFFFF)
![CPP](https://img.shields.io/badge/C%2B%2B-C17-GREEN?style=for-the-badge&logo=C%2B%2B&logoColor=%23FFFFFF)
### Project
![Project](https://img.shields.io/badge/KEC-blue?style=for-the-badge)

### Struct
```
├── route_tracker
│    ├── include
│    ├── src
│    ├── launch
│    └── param
└── route_msgs
     ├── action
     └── msg
```

### Build
```
 1. 작업 전 의존 패키지 목록
        rclcpp
        rclcpp_action
        route_msgs
        std_msgs
        sensor_msgs
        tf2
        geometry_msgs
        nav_msgs
        routedevation_msgs
        obstacle_msgs
        robot_status_msgs
        can_msgs

2. Build
    $ colcon build --install--symlink
    $ ros2 interface list | grep route
```

### Parameter Setting 
```
    $ vi param kec_route_trakcer.yaml
```

### Run
```
    $ source install/setup.bash
    $ ros2 launch route_trakcer route_tracker_launch.py
```

### Data
#### 1. Topic
##### 1.1. Publish
```
* /cmd_vel
* /drive/break
* /drive/info
* /drive/can/ad_control_body
* /drive/obstacle/event
* /drive/odom/offset
* /route_tracker_log
```
##### 1.2. Subscribe
```
* /sensor/imu/data
* /sensor/ublox/fix
* /drive/odom/origin
* /drive/odom/eular
* /drive/route_deviation/status
* /drive/obstacle/status
* /drive/velocity/state
```
#### 2. Action
##### 2.1 Goal Service
```
* /drive/route_to_pose
```
##### 2.2 Action Struct
```
├── Tree
     ├── Action Goal
     ├── Action Result
     └── Action FeedBack
```