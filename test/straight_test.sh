ros2 topic pub -1 /sensor/ublox/fix sensor_msgs/msg/NavSatFix "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
status:
  status: 0
  service: 0
latitude: 37.3059883
longitude: 127.2401595
altitude: 0.0
position_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
position_covariance_type: 0"

ros2 topic pub -1 /drive/route_deviation/status routedevation_msgs/msg/Status "offcource_status: false
offcource_out_distance: 0.0
offcource_goal_distance: 0.0
offcource_start_lat: 0.0
offcource_start_lon: 0.0
offcource_dest_lat: 0.0
offcource_dest_lon: 0.0
offcource_goal_lat: 0.0
offcource_goal_lon: 0.0
offcource_goal_x: 0.0
offcource_goal_y: 0.0"

