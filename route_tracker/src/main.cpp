#ifndef ROUTE_TRACKER_MAIN_
#define ROUTE_TRACKER_MAIN_


#include "route_tracker/center.hpp"

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    //rclcpp::spin(std::make_shared<Center>());
    rclcpp::executors::MultiThreadedExecutor executor;
    auto center =std::make_shared<Center>();
    executor.add_node(center);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

#endif