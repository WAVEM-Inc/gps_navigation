//
// Created by nuc-bt on 24. 3. 5.
//

#include "route_tracker/center.hpp"

Center::Center() : Node("route_tracker_node") {
    // action_server
    rclcpp::CallbackGroup::SharedPtr callback_group_action_server;
    callback_group_action_server = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    route_to_pose_action_server_ = rclcpp_action::create_server<RouteToPose>(
            this,
            "/route_to_pose",
            std::bind(&Center::route_to_pose_goal_handle, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&Center::route_to_pose_cancel_handle, this, std::placeholders::_1),
            std::bind(&Center::route_to_pose_accepted_handle, this, std::placeholders::_1),
            rcl_action_server_get_default_options(),
            callback_group_action_server
            );
    // imu callback
    // gps callback
    // 장애물 정보 callback
    // 경로 이탈 정보 callback
    // 로봇 모드 timer
    //   ㄴ speaker timer
}


void Center::fn_run() {

}

rclcpp_action::GoalResponse
Center::route_to_pose_goal_handle(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const RouteToPose::Goal> goal) {
    return rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse
Center::route_to_pose_cancel_handle(const std::shared_ptr<RouteToPoseGoalHandler> goal_handle) {
    return rclcpp_action::CancelResponse::REJECT;
}

void Center::route_to_pose_accepted_handle(const std::shared_ptr<RouteToPoseGoalHandler> goal_handle) {

}

void Center::route_to_pose_execute(const std::shared_ptr<RouteToPoseGoalHandler> goal_handler) {

}
