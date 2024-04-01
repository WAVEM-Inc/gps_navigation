# gps_navigation
auto-driving package with gps

### DEV
![FrameWork](https://img.shields.io/badge/ROS2-orange?style=for-the-badge&logo=ros&logoColor=white)

### Project
![Project](https://img.shields.io/badge/KEC-blue?style=for-the-badge)

### Struct
* KEC
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

2. Build
    $ colcon build --install--symlink
    $ ros2 interface list | grep route
```

### Setting 
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
```
##### 1.2. Subscribe
```
* /sensor/imu/data
* /sensor/ublox/fix
* /odom
* /drive/route_deviation/status
* /drive/obstacle/status
* /drive/velocity/state

```
#### 2. Action
##### 2.1 Goal Service
```
* /drive/route_to_pose
```