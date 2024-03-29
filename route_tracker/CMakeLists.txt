cmake_minimum_required(VERSION 3.8)
project(route_tracker)
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(route_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(routedevation_msgs REQUIRED)
find_package(obstacle_msgs REQUIRED)
find_package(robot_status_msgs REQUIRED)
find_package(PROJ REQUIRED CONFIG)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)
include_directories(${GeographicLib_INCLUDE_DIRS})

include_directories(
        include
        include/${PROJECT_NAME}
        include/entity
        include/route_tracker
        include/math
        include/common
        #${EXTERNAL_INCLUDE_DIRS}
)

set(dependencies
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
)

set(source_list
        src/main.cpp
        src/math/distance.cpp
        include/math/distance.hpp
        src/entity/gps_data.cpp
        include/entity/gps_data.hpp
        src/entity/gps_data.cpp
        src/center.cpp
        include/route_tracker/center.hpp
        include/common/constants.hpp
        src/math/imu_convert.cpp
        include/math/imu_convert.hpp
        include/common/ros_parameter.hpp
        include/code/kec_car_data.hpp
        include/code/kec_car_data.hpp
        include/code/kec_car_data.hpp
        src/entity/car.cpp
        include/entity/car.hpp
        include/code/kec_driving_data_code.hpp
        include/math/geo_trans.hpp
        include/common/data_type_trans.hpp
        src/entity/task.cpp
        include/entity/task.hpp
        src/math/car_behavior.cpp
        include/math/car_behavior.hpp
        include/common/test.h
)

set(node_name
        route_tracker_node
)

add_executable(${node_name}
        ${source_list}
)

ament_target_dependencies(${node_name}
        ${dependencies}
)

add_definitions(-DACCEPT_USE_OF_DEPRECATED_PROJ_API_H)

target_link_libraries (${node_name}
        ${GeographicLib_LIBRARIES}
	PROJ::proj
)
install(TARGETS
        ${node_name}
        DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
        param
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
