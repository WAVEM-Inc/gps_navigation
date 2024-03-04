cmake_minimum_required(VERSION 3.8)
project(route_tracker)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(route_msgs REQUIRED)

include_directories(
        include
        include/${PROJECT_NAME}
        include/entity

        #${EXTERNAL_INCLUDE_DIRS}
)

set(dependencies
  rclcpp
  route_msgs
  std_msgs
)

set(source_list
        src/main.cpp
        src/center.cpp
        include/route_tracker/center.hpp
        src/math/distance.cpp
        include/math/distance.hpp
        src/entity/gps_data.cpp
        include/entity/gps_data.hpp
        src/entity/gps_data.cpp
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

ament_package()
