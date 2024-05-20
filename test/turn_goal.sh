ros2 action send_goal /route_to_pose route_msgs/action/RouteToPose "start_node:
  node_id: '201'
  position:
    latitude: 37.3061471
    longitude: 127.2401769
  type: ''
  kind: 'connecting'
  heading: 130.0
  direction: ''
  driving_option: ''
  detection_range: []
end_node:
  node_id: '202'
  position:
    latitude: 37.3061047
    longitude: 127.2401450
  type: ''
  kind: 'waiting'
  heading: 130.0
  direction: ''
  driving_option: ''
  detection_range: []"

ros2 action send_goal /route_to_pose route_msgs/action/RouteToPose "start_node:
  node_id: '202'
  position:
    latitude: 37.3061047
    longitude: 127.2401450
  type: ''
  kind: 'waiting'
  heading: 130.0
  direction: ''
  driving_option: ''
  detection_range: []
end_node:
  node_id: '203'
  position:
    latitude: 37.3060612
    longitude: 127.2401142
  type: ''
  kind: 'intersection'
  heading: 220.0
  direction: ''
  driving_option: ''
  detection_range: []"



