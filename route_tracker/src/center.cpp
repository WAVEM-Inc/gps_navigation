//
// Created by nuc-bt on 24. 3. 5.
//

#include "route_tracker/center.hpp"

Center::Center() : Node("route_tracker_node") {
    // action_server

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

void Center::handle_accepted(const std::shared_ptr<RouteToPoseGoalHandler> goal_handle) {

}
