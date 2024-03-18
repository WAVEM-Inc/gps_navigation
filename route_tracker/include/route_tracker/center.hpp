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
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
// custom msg
#include "route_msgs/action/route_to_pose.hpp"
#include "route_msgs/msg/drive_state.hpp"
#include "route_msgs/msg/drive_break.hpp"
#include "routedevation_msgs/msg/status.hpp"
#include "obstacle_msgs/msg/status.hpp"

//
#include "common/constants.hpp"
#include "common/ros_parameter.hpp"
#include "math/imu_convert.hpp"
#include "entity/car.hpp"
#include "task.hpp"

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

private :
    //field
    using RouteToPose = route_msgs::action::RouteToPose;
    using RouteToPoseGoalHandler = rclcpp_action::ServerGoalHandle<RouteToPose>;
    rclcpp_action::Server<RouteToPose>::SharedPtr route_to_pose_action_server_;

    // field subscribe
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<routedevation_msgs::msg::Status>::SharedPtr sub_route_deviation_;
    rclcpp::Subscription<obstacle_msgs::msg::Status>::SharedPtr sub_obstacle_status_;

    //field publish
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::Publisher<route_msgs::msg::DriveBreak>::SharedPtr pub_break_;
    rclcpp::Publisher<route_msgs::msg::DriveState>::SharedPtr pub_drive_state_;
    //field timer
    rclcpp::TimerBase::SharedPtr timer_drive_state_;

    //
    std::unique_ptr<Constants> constants_;
    /// field action
    std::shared_ptr<route_msgs::msg::Node> cur_node_;
    std::shared_ptr<route_msgs::msg::Node> next_node_;
    // field entity
    std::unique_ptr<RosParameter> ros_parameter_;
    // field math
    std::unique_ptr<ImuConvert> imu_converter_;
    // field data
    std::unique_ptr<Car> car_;
    std::unique_ptr<TaskGoal> task_;

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
    void route_to_pose_execute(const std::shared_ptr<RouteToPoseGoalHandler> goal_handle);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu);
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr gps);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom);
    void route_deviation_callback(const routedevation_msgs::msg::Status::SharedPtr status);
    void obstacle_status_callback(const obstacle_msgs::msg::Status::SharedPtr status);
    //
    void drive_info_timer();
    //
    kec_car::NodeKind car_mode_determine(std::string car_node);
    geometry_msgs::msg::Twist calculate_straight_movement(float acceleration);
};


#endif //ROUTE_TRACKER_CENTER_HPP
