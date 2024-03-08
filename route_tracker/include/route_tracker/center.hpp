//
// Created by nuc-bt on 24. 3. 5.
//

#ifndef ROUTE_TRACKER_CENTER_HPP
#define ROUTE_TRACKER_CENTER_HPP
// default
#include<iostream>
// ros2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
// custom msg
#include "route_msgs/action/route_to_pose.hpp"
#include "route_msgs/msg/drive_state.hpp"
#include "entity/constants.hpp"
#include "entity/ros_parameter.hpp"
#include "math/imu_convert.hpp"
#include "car.hpp"

/**
 * @brief
 *  [ ] action_server
 *  [V] imu callback
 *  [V] gps callback
 *  [ ] 장애물 정보 callback
 *  [ ] 경로 이탈 정보 callback
 *  [ ] 로봇 모드 timer
           ㄴ speaker timer
 */
class Center : public rclcpp::Node{
public :
    explicit Center();
    void fn_run();
private :
    //field
    using RouteToPose = route_msgs::action::RouteToPose;
    using RouteToPoseGoalHandler = rclcpp_action::ServerGoalHandle<RouteToPose>;
    rclcpp_action::Server<RouteToPose>::SharedPtr route_to_pose_action_server_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_;
    std::unique_ptr<Constants> constants_;
    /// field action
    std::shared_ptr<route_msgs::msg::Node> cur_node_;
    std::shared_ptr<route_msgs::msg::Node> prev_node_;
    // field entity
    std::unique_ptr<RosParameter> ros_parameter_;
    // field math
    std::unique_ptr<ImuConvert> imu_converter_;
    // field data
    std::unique_ptr<Car> car_;

    //function
    void ros_parameter_setting();
    void ros_init();

    rclcpp_action::GoalResponse route_to_pose_goal_handle(
            const rclcpp_action::GoalUUID& uuid,
            std::shared_ptr<const RouteToPose::Goal> goal
            );
    rclcpp_action::CancelResponse route_to_pose_cancel_handle(
            const std::shared_ptr<RouteToPoseGoalHandler> goal_handle
            );
    void route_to_pose_accepted_handle(
            const std::shared_ptr<RouteToPoseGoalHandler> goal_handle
            );
    void route_to_pose_execute(const std::shared_ptr<RouteToPoseGoalHandler> goal_handler);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu);
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr gps);

    kec_car::NodeKind car_mode_determine(std::string car_node);
};


#endif //ROUTE_TRACKER_CENTER_HPP
