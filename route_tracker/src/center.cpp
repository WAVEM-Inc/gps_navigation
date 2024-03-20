//
// Created by nuc-bt on 24. 3. 5.
//

#include "route_tracker/center.hpp"
#include "code/kec_driving_data_code.hpp"
#include "distance.hpp"
#include "common/data_type_trans.hpp"
#include "car_behavior.hpp"

#define FIRST_ZONE 0.3
#define SECOND_ZONE 0.6
#define SETTING_ZERO 0.0
#define MAX_DIRECTION 45

Center::Center() : Node("route_tracker_node") {
        constants_ = std::make_unique<Constants>();
        imu_converter_ = std::make_unique<ImuConvert>();
        car_ = std::make_unique<Car>();
        ros_parameter_setting();
        ros_init();
}

/**
 *
 */
void Center::ros_init() {
        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
        // action_server
        rclcpp::CallbackGroup::SharedPtr callback_group_action_server = this->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);
        route_to_pose_action_server_ = rclcpp_action::create_server<RouteToPose>(
                this,
                constants_->tp_name_route_to_pose_,
                std::bind(&Center::route_to_pose_goal_handle, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&Center::route_to_pose_cancel_handle, this, std::placeholders::_1),
                std::bind(&Center::route_to_pose_accepted_handle, this, std::placeholders::_1),
                rcl_action_server_get_default_options(),
                callback_group_action_server
        );
        //==
        // subscribe
        //==
        // imu callback
        rclcpp::CallbackGroup::SharedPtr cb_group_imu;
        cb_group_imu = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions sub_imu_options;
        sub_imu_options.callback_group = cb_group_imu;
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(constants_->tp_name_imu_, default_qos,
                                                                    std::bind(&Center::imu_callback, this,
                                                                              std::placeholders::_1), sub_imu_options);
        // gps callback
        rclcpp::CallbackGroup::SharedPtr cb_group_gps;
        cb_group_gps = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions sub_gps_options;
        sub_gps_options.callback_group = cb_group_gps;
        sub_gps_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(constants_->tp_name_gps_, default_qos,
                                                                          std::bind(&Center::gps_callback, this,
                                                                                    std::placeholders::_1),
                                                                          sub_gps_options);

        // odom callback
        rclcpp::CallbackGroup::SharedPtr cb_group_odom;
        cb_group_odom = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions sub_odom_options;
        sub_odom_options.callback_group = cb_group_odom;
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(constants_->tp_name_odom_, default_qos,
                                                                       std::bind(&Center::odom_callback, this,
                                                                                 std::placeholders::_1),
                                                                       sub_odom_options);
        // route_deviation
        // 경로 이탈
        rclcpp::CallbackGroup::SharedPtr cb_group_route_deviation;
        cb_group_route_deviation = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions sub_route_deviation_options;
        sub_route_deviation_options.callback_group = cb_group_route_deviation;
        sub_route_deviation_ = this->create_subscription<routedevation_msgs::msg::Status>(
                constants_->tp_name_route_deviation_,
                default_qos,
                std::bind(&Center::route_deviation_callback,
                          this,
                          std::placeholders::_1),
                sub_route_deviation_options);

        // 장애물 정보
        // obstacle_status
        rclcpp::CallbackGroup::SharedPtr cb_group_obstacle_status;
        cb_group_obstacle_status = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions sub_obstacle_status_options;
        sub_obstacle_status_options.callback_group = cb_group_obstacle_status;
        sub_obstacle_status_ = this->create_subscription<obstacle_msgs::msg::Status>(
                constants_->tp_name_obstacle_status_,
                default_qos,
                std::bind(&Center::obstacle_status_callback,
                          this,
                          std::placeholders::_1),
                sub_obstacle_status_options);

        //==
        // publisher
        //==
        // cmd_vel (이동 정보)
        rclcpp::CallbackGroup::SharedPtr cb_group_cmd;
        cb_group_cmd = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::PublisherOptions pub_cmd_options;
        pub_cmd_options.callback_group = cb_group_cmd;
        pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>(constants_->tp_name_cmd_, default_qos,
                                                                     pub_cmd_options);
        // 브레이크 작동
        // 필요시 callback group 이용할 것.
        // 테스트하여 브레이크 되는지 보고 사용할 것.
        pub_break_ = this->create_publisher<route_msgs::msg::DriveBreak>(constants_->tp_name_drive_break_,
                                                                         default_qos);

        // drive state - 스피커, 주행 모드
        pub_drive_state_ = this->create_publisher<route_msgs::msg::DriveState>(
                constants_->tp_name_drive_info_,
                default_qos);
        // 로봇 모드 timer
        //   ㄴ speaker timer
        // 0.1 sec = 100ms
        rclcpp::CallbackGroup::SharedPtr drive_info_callback_group = this->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_drive_state_ = this->create_wall_timer(std::chrono::seconds(1),
                                                     std::bind(&Center::drive_info_timer,
                                                               this),
                                                     drive_info_callback_group);
}

/**
 * @brief Route_to_pose goal reception function
 * @see route_to_pose_accepted_handle
 * @see route_to_pose_execute
 * @date 24-03-19
 * @param uuid
 * @param goal
 * @return rclcpp_action::GoalResponse
 */
rclcpp_action::GoalResponse
Center::route_to_pose_goal_handle(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const RouteToPose::Goal> goal) {
        // 1) goal 수신
        // 2) route_to_pose_goal_handle 호출
        task_ = std::make_unique<TaskGoal>(goal->start_node, goal->end_node);

        // * [Exception Handling] 연결 노드 일때 45도 이상 전환하지 못하도록
        try {
                CarBehavior car_behavior;
                //4) 직진 명령인가?
                if (car_behavior.straight_judgment(task_->get_cur_node_kind(), task_->get_next_node_kind())) {
                        // abs(출발지 노드 진출 방향-기존 노드 진출 방향) >45
                        // 4-Y) 각도가 예외 상황인가?
                        // 4-Y-Y) REJECT
                        // 4-Y-N) route_to_pose_accepted_handle
                        return std::abs(car_->get_degree() - task_->get_cur_heading()) > MAX_DIRECTION
                               ? rclcpp_action::GoalResponse::REJECT
                               : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
                }
        } //try
        catch (std::out_of_range &error) {
                // 3) 올바른 이동 명령인가?
                return rclcpp_action::GoalResponse::REJECT;
        } // catch
        // 4-N) route_to_pose_accepted_handle
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
Center::route_to_pose_cancel_handle(const std::shared_ptr<RouteToPoseGoalHandler> goal_handle) {
        return rclcpp_action::CancelResponse::ACCEPT;
}

/**
 * @brief A function that operates route_to_pose_execute as a thread, responsible for the core functionality of handling goals
 * @param goal_handle
 */
void Center::route_to_pose_accepted_handle(const std::shared_ptr<RouteToPoseGoalHandler> goal_handle) {
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        // 5) route_to_pose_execute
        std::thread{std::bind(&Center::route_to_pose_execute, this, std::placeholders::_1), goal_handle}.detach();
}

void Center::route_to_pose_execute(const std::shared_ptr<RouteToPoseGoalHandler> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        // max speed
        rclcpp::Rate loop_rate(1);
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<RouteToPose::Feedback>();
        auto result = std::make_shared<RouteToPose::Result>();
        std::unique_ptr<Distance> center_distance = std::make_unique<Distance>();
        CarBehavior car_behavior;
        while (rclcpp::ok()) {
                // [Cancel]
                //5-1) 취소 명령이 있는가?
                if (goal_handle->is_canceling()) {
                        result->result = static_cast<int>(kec_driving_code::Result::kFailedCancel);
                        goal_handle->canceled(result);
                        RCLCPP_INFO(this->get_logger(), "Goal canceled");
                        return;
                }
                // [Exception Handling]
                //5-2) 센서 데이터가 들어오는가?
                if (car_->get_location().fn_get_longitude() == 0 && car_->get_location().fn_get_latitude() == 0) {
                        feedback->status_code = static_cast<int>(kec_driving_code::FeedBack::kSensorWaiting);
                        goal_handle->publish_feedback(feedback);
                        RCLCPP_INFO(this->get_logger(), "Publish feedback");
                        continue;
                }

                //6-1) 진진 주행
                if (car_behavior.straight_judgment(task_->get_cur_node_kind(), task_->get_next_node_kind())) {
                        // 목적지 도착 여부
                        // distance_from_perpendicular_line();
                        GpsData start_node_position = task_->get_cur_gps();
                        GpsData end_node_position = task_->get_next_gps();
                        GpsData cur_location(car_->get_location().fn_get_latitude(),
                                             car_->get_location().fn_get_longitude());
                        // 도착지 거리
                        // 파라미터 작업
                        if (center_distance->distance_from_perpendicular_line(start_node_position, end_node_position,
                                                                              cur_location) <=
                            ros_parameter_->goal_distance_) {
                                break;
                        }
                        // 이벤트 판단

                        //
                        // 직진 주행 명령
                        geometry_msgs::msg::Twist cmd_vel = calculate_straight_movement(ros_parameter_->max_speed_);
                        pub_cmd_->publish(cmd_vel);
                        // if 도착 명령 확인
                        // else
                        feedback->status_code = static_cast<int>(kec_driving_code::FeedBack::kStart);
                        goal_handle->publish_feedback(feedback);
                        RCLCPP_INFO(this->get_logger(), "Publish feedback");
                }
                //6-2) 회전 주행
                else if (car_behavior.intersection_judgment(task_->get_cur_node_kind(),
                                                                 task_->get_next_node_kind())) {
                        // 2-5) 장애물 존재 여부 확인
                        if (!obs_status_->obstacle_value) {
                                //obstacle_value = ture - 감지, false- 미감지
                                continue;
                        } else {
                                double goal_distance = center_distance->distance_from_perpendicular_line(
                                        task_->get_cur_gps(), task_->get_next_gps(), car_->get_location());
                                // 2-6) 직진 목적지에 도착했는가?
                                // 회전 중 틀어지지 않도록 task_->rotation.. 을 통해 목적지 판단 무시
                                // 최초에는 목적지 도착하여 if문, 이후에는 check를 통해 진입
                                if (goal_distance < ros_parameter_->rotation_straight_dist_ || (task_->rotation_straight_check_)) {
                                        // 2-6-1) 도착함
                                        // 2-7) 교차로 노드 헤딩 정보로 방향 확인
                                        task_->rotation_straight_check_= true;
                                        double next_node_heading = task_->get_next_heading();
                                        double car_heading = car_->get_degree();
                                        // 2-8) 회전 여부 확인
                                        if(!car_behavior.car_rotation_judgment(std::abs(car_heading-next_node_heading),ros_parameter_->rotation_angle_tolerance_)) {
                                                // 2-9) IMU 각도 교차로 헤딩을 통한 방향 확인
                                                if (car_behavior.car_move_direct(car_->get_degree(),
                                                                                 task_->get_next_heading())) {
                                                        //right
                                                        geometry_msgs::msg::Twist cmd_vel = car_behavior.calculate_rotation_movement(
                                                                0.1, ros_parameter_->rotation_angle_increase_);
                                                        pub_cmd_->publish(cmd_vel);
                                                } else {
                                                        //left
                                                        geometry_msgs::msg::Twist cmd_vel = car_behavior.calculate_rotation_movement(
                                                                0.1, -ros_parameter_->rotation_angle_increase_);
                                                        pub_cmd_->publish(cmd_vel);
                                                }
                                        }
                                        else{
                                                // 2-9) 종료
                                                break;
                                        }

                                } else {
                                        //2-6-2) 도착하지 않음
                                        geometry_msgs::msg::Twist cmd_vel = calculate_straight_movement(ros_parameter_->max_speed_);
                                        pub_cmd_->publish(cmd_vel);
                                } // 2-6-2)
                        }// 2-5)
                }

        } // while

        // 7) 완료 처리
        if (rclcpp::ok()) {
                result->result = static_cast<int>(kec_driving_code::Result::kSuccess);
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
}

/**
 * @brief imu callback, 차량 각도 제공
 * @param imu
 */
void Center::imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu) {
        std::lock_guard<std::mutex> lock(mutex_);
        imu_converter_->set_correction(ros_parameter_->imu_correction_);
        car_->set_degree(static_cast<float>(imu_converter_->quaternion_to_heading_converter(imu)));
}

void Center::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr gps) {
        std::lock_guard<std::mutex> lock(mutex_);
        GpsData data(gps->latitude, gps->longitude);
        car_->set_location(data);
}

void Center::ros_parameter_setting() {
        this->declare_parameter<float>("imu_correction", SETTING_ZERO);
        this->declare_parameter<float>("max_speed", SETTING_ZERO);
        this->declare_parameter<float>("driving_calibration_max_angle", SETTING_ZERO);
        this->declare_parameter<float>("driving_calibration_min_angle", SETTING_ZERO);
        this->declare_parameter<float>("driving_calibration_angle_increase", SETTING_ZERO);
        this->declare_parameter<float>("goal_distance", SETTING_ZERO);
        this->declare_parameter<float>("rotation_straight_dist", SETTING_ZERO);
        this->declare_parameter<float>("rotation_angle_increase",SETTING_ZERO);
        this->declare_parameter<float>("rotation_angle_tolerance",SETTING_ZERO);
        float imu_correction;
        float max_speed;
        float driving_calibration_min_angle;
        float driving_calibration_max_angle;
        float driving_calibration_angle_increase;
        float goal_distance;
        float rotation_straight_dist;
        float rotation_angle_increase;
        float rotation_angle_tolerance;
        this->get_parameter("imu_correction", imu_correction);
        this->get_parameter("max_speed", max_speed);
        this->get_parameter("driving_calibration_angle", driving_calibration_min_angle);
        this->get_parameter("driving_calibration_angle", driving_calibration_max_angle);
        this->get_parameter("driving_calibration_angle_increase", driving_calibration_angle_increase);
        this->get_parameter("goal_distance", goal_distance);
        this->get_parameter("rotation_straight_dist", rotation_straight_dist);
        this->get_parameter("rotation_angle_increase",rotation_angle_increase);
        this->get_parameter("rotation_angle_tolerance",rotation_angle_tolerance);
        ros_parameter_ = std::make_unique<RosParameter>(imu_correction,
                                                        max_speed,
                                                        driving_calibration_max_angle,
                                                        driving_calibration_min_angle,
                                                        driving_calibration_angle_increase,
                                                        goal_distance,
                                                        rotation_straight_dist,
                                                        rotation_angle_increase,
                                                        rotation_angle_tolerance);
}


geometry_msgs::msg::Twist Center::calculate_straight_movement(float acceleration) {
        geometry_msgs::msg::Twist result;
        result.linear.set__x(acceleration);
        result.linear.set__y(SETTING_ZERO);
        result.linear.set__z(SETTING_ZERO);
        // 현재 각도-노드진입 탈출 각도를 통하여 링크와 얼마나 다른 방향으로 가는지 확인
        const double link_degree = car_->get_degree() - task_->get_cur_heading();
        // 각도에 따라 더 틀지 결정
        const double first_zone = ros_parameter_->driving_calibration_max_angle_ * FIRST_ZONE;
        const double second_zone = ros_parameter_->driving_calibration_max_angle_ * SECOND_ZONE;
        // 왼쪽/ 오른쪽
        const double direction = (link_degree > 0) ? ros_parameter_->driving_calibration_angle_increase_
                                                   : -ros_parameter_->driving_calibration_angle_increase_;
        if (link_degree > ros_parameter_->driving_calibration_min_angle_) {
                if (link_degree < first_zone) {
                        result.angular.set__z(direction);
                } else if (link_degree > first_zone &&
                           link_degree < second_zone) {
                        result.angular.set__z(direction * 2);
                } else if (link_degree > second_zone) {
                        result.angular.set__z(direction * 3);
                }
        } else {
                //직선 주행
                result.angular.set__x(SETTING_ZERO);
                result.angular.set__y(SETTING_ZERO);
                result.angular.set__z(SETTING_ZERO);
        }
        return result;
}

void Center::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom) {

}

void Center::route_deviation_callback(const routedevation_msgs::msg::Status::SharedPtr status) {

}

/**
 * @brief obstacle_status : 0 - drive, 1 - intersection, 2 - parking
 * @brief obstacle_value :
 * @param status
 */
void Center::obstacle_status_callback(const obstacle_msgs::msg::Status::SharedPtr status) {
        obs_status_ = status;
}

void Center::drive_info_timer() {
        DataTypeTrans data_type_trans;
        route_msgs::msg::DriveState drive_state;
        drive_state.code = data_type_trans.drive_mode_to_string(car_->get_drive_mode());
        drive_state.speaker;
        pub_drive_state_->publish(drive_state);
}
