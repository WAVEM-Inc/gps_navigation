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

// custom msg
#include "route_msgs/action/route_to_pose.hpp"
/**
 * @brief
 *  [ ] action_server
    [ ] imu callback
    [ ] gps callback
    [ ] 장애물 정보 callback
    [ ] 경로 이탈 정보 callback
    [ ] 로봇 모드 timer
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
    rclcpp_action::Server<RouteToPose> route_to_pose_action_server_;

    //function
    rclcpp_action::GoalResponse route_to_pose_goal_handle(
            const rclcpp_action::GoalUUID& uuid,
            std::shared_ptr<const RouteToPose::Goal> goal
            );
    rclcpp_action::CancelResponse route_to_pose_cancel_handle(
            const std::shared_ptr<RouteToPoseGoalHandler> goal_handle
            );
    void handle_accepted(
            const std::shared_ptr<RouteToPoseGoalHandler> goal_handle
            );
};


#endif //ROUTE_TRACKER_CENTER_HPP
