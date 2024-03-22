cmake_minimum_required(VERSION 3.8)
project(route_msgs)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set( msg_files
        "msg/DetectionRange.msg"
        "msg/Link.msg"
        "msg/Node.msg"
        "msg/Path.msg"
        "msg/Position.msg"
        "msg/DriveState.msg"
        "msg/DriveBreak.msg"
        "action/RouteToPose.action"
)
rosidl_generate_interfaces(${PROJECT_NAME}
        ${msg_files}
        DEPENDENCIES builtin_interfaces std_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()