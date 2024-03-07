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
find_package(tf2 REQUIRED)

include_directories(
        include
        include/${PROJECT_NAME}
        include/entity
        include/route_tracker
        include/math
        #${EXTERNAL_INCLUDE_DIRS}
)

set(dependencies
        rclcpp
        rclcpp_action
        route_msgs
        std_msgs
        sensor_msgs
        tf2
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
        include/entity/constants.hpp
        src/math/imu_convert.cpp
        include/math/imu_convert.hpp
        include/entity/ros_parameter.hpp
        include/entity/kec_car_data.hpp
        include/entity/kec_car_data.hpp
        include/entity/kec_car_data.hpp
        src/entity/car.cpp
        include/entity/car.hpp
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
