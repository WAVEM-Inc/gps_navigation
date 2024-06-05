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
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

// custom msg
#include "route_msgs/action/route_to_pose.hpp"
#include "route_msgs/msg/drive_state.hpp"
#include "route_msgs/msg/drive_break.hpp"
#include "routedevation_msgs/msg/status.hpp"
#include "obstacle_msgs/msg/status.hpp"
#include "robot_status_msgs/msg/velocity_status.hpp"
#include "can_msgs/msg/ad_control_body.hpp"

//
#include "common/constants.hpp"
#include "common/ros_parameter.hpp"
#include "math/imu_convert.hpp"
#include "entity/car.hpp"
#include "task.hpp"
#include "math/car_behavior.hpp"
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
    double init_distance ;
    bool cancel_check_;
    bool reject_check_;
    //field
    using RouteToPose = route_msgs::action::RouteToPose;
    using RouteToPoseGoalHandler = rclcpp_action::ServerGoalHandle<RouteToPose>;
    rclcpp_action::Server<RouteToPose>::SharedPtr route_to_pose_action_server_;

    // field subscribe
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_odom_eular_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<routedevation_msgs::msg::Status>::SharedPtr sub_route_deviation_;
    rclcpp::Subscription<obstacle_msgs::msg::Status>::SharedPtr sub_obstacle_status_;
    rclcpp::Subscription<robot_status_msgs::msg::VelocityStatus>::SharedPtr sub_velocity_status_;

    //field publish
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::Publisher<route_msgs::msg::DriveBreak>::SharedPtr pub_break_;
    rclcpp::Publisher<route_msgs::msg::DriveState>::SharedPtr pub_drive_state_;
    rclcpp::Publisher<can_msgs::msg::AdControlBody>::SharedPtr pub_body_;
    rclcpp::Publisher<obstacle_msgs::msg::Status>::SharedPtr pub_obs_event_;
    //field timer
    rclcpp::TimerBase::SharedPtr timer_drive_state_;
    rclcpp::TimerBase::SharedPtr timer_ptr_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
    rclcpp::CallbackGroup::SharedPtr cbg_action_server_ ;
    rclcpp::CallbackGroup::SharedPtr cbg_drive_info_timer_ ;
    rclcpp::CallbackGroup::SharedPtr cbg_cmd_;
    rclcpp::CallbackGroup::SharedPtr cbg_break_;
    rclcpp::CallbackGroup::SharedPtr cbg_drive_info_pub_;
    rclcpp::CallbackGroup::SharedPtr cbg_velocity_;
    rclcpp::CallbackGroup::SharedPtr cbg_obstacle_status_;
    rclcpp::CallbackGroup::SharedPtr cbg_route_deviation_;
    rclcpp::CallbackGroup::SharedPtr cbg_odom_;
    rclcpp::CallbackGroup::SharedPtr cbg_gps_;
    rclcpp::CallbackGroup::SharedPtr cbg_imu_;
    rclcpp::CallbackGroup::SharedPtr cbg_odom_euler_;
    rclcpp::CallbackGroup::SharedPtr cbg_pub_body_;
    rclcpp::CallbackGroup::SharedPtr cbg_pub_obs_event_;
    bool feedback_check_;
    bool obstacle_first_check_;
    int speaker_seq_;
    //
    std::unique_ptr<Constants> constants_;
    // field entity
    std::unique_ptr<RosParameter> ros_parameter_;
    // field math
    std::unique_ptr<ImuConvert> imu_converter_;
    // field data
    std::unique_ptr<Car> car_;
    std::unique_ptr<TaskGoal> task_;
    //
    std::mutex mutex_;
    std::mutex goal_mutex_;
    std::condition_variable cv_;
    bool waiting_check_;
    float prev_speed_; // 가속을 위함.
    std::shared_ptr<obstacle_msgs::msg::Status> obs_status_;
    std::shared_ptr<routedevation_msgs::msg::Status> devation_status_;
    std::shared_ptr<obstacle_msgs::msg::Status> prev_status_;
    //function
    void ros_parameter_setting();
    void ros_init();
    void cmd_stop();
    float speed_setting(float goal_dist, float brake_dist);
    float speed_setting(const float goal_dist, const float init_dist, const float brake_dist);
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
    void odom_eular_callback(const geometry_msgs::msg::PoseStamped::SharedPtr odom_eular);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom);
    void route_deviation_callback(const routedevation_msgs::msg::Status::SharedPtr status);
    void obstacle_status_callback(const obstacle_msgs::msg::Status::SharedPtr status);
    void velocity_status_callback(const robot_status_msgs::msg::VelocityStatus::SharedPtr status);
    //
    void drive_info_timer();
    //
    void brake_unlock();
    void calculate_straight_movement(float acceleration);
    void start_on(const std::shared_ptr<RouteToPose::Feedback> feedback,const std::shared_ptr<RouteToPoseGoalHandler> goal_handle);
    bool cancel_check(const std::shared_ptr<RouteToPose::Result>result , const std::shared_ptr<RouteToPoseGoalHandler>goal_handle);
    void car_rotation(CarBehavior car_behavior,double node_heading, kec_car::NodeKind node_kind, double init);
    void straight_move(const std::shared_ptr<RouteToPose::Feedback> feedback,const std::shared_ptr<RouteToPose::Result>result , const std::shared_ptr<RouteToPoseGoalHandler>goal_handle ,CarBehavior car_behavior );
    void straight_move_correction(float acceleration);
    void odom_move(const std::shared_ptr<RouteToPose::Feedback> feedback,const std::shared_ptr<RouteToPose::Result>result , const std::shared_ptr<RouteToPoseGoalHandler>goal_handle);
    void turn_move(const std::shared_ptr<RouteToPose::Feedback> feedback,const std::shared_ptr<RouteToPose::Result>result , const std::shared_ptr<RouteToPoseGoalHandler>goal_handle,CarBehavior car_behavior);
    void recovery_move(routedevation_msgs::msg::Status devation_status,const std::shared_ptr<RouteToPose::Feedback> feedback,
                           const std::shared_ptr<RouteToPose::Result> result,
                           const std::shared_ptr<RouteToPoseGoalHandler> goal_handle,
                           kec_car::DrivingMode mode);
};


#endif //ROUTE_TRACKER_CENTER_HPP
