//
// Created by nuc-bt on 24. 3. 5.
//

#include "route_tracker/center.hpp"
#include "code/kec_driving_data_code.hpp"
#include "distance.hpp"
#include "common/data_type_trans.hpp"

#include "common/test.h"

#define FIRST_ZONE 0.3
#define SECOND_ZONE 0.6
#define SETTING_ZERO 0.0
#define MAX_DIRECTION 45
using namespace std::chrono_literals;

Center::Center() : Node("route_tracker_node"),feedback_check_(false),waiting_check_(false) {
    constants_ = std::make_unique<Constants>();
    imu_converter_ = std::make_unique<ImuConvert>();
    car_ = std::make_unique<Car>();
    car_->set_speed(0);
    ros_parameter_setting();
    ros_init();
}

/**
 *
 */
void Center::ros_init() {
    obstacle_msgs::msg::Status init_obstacle;
    init_obstacle.obstacle_status = 0;
    init_obstacle.obstacle_value = false;
    //
    routedevation_msgs::msg::Status init_routedevation;
    init_routedevation.offcource_status = 0;
    //
    GpsData data(0, 0);

    car_->set_location(data);
    car_->set_degree(0);
    car_->set_drive_mode(kec_car::DrivingMode::kStop);
    obs_status_ = std::make_shared<obstacle_msgs::msg::Status>(init_obstacle);
    devation_status_ = std::make_shared<routedevation_msgs::msg::Status>(init_routedevation);


    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    // action_server
    cbg_action_server_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);
    route_to_pose_action_server_ = rclcpp_action::create_server<RouteToPose>(
            this,
            constants_->tp_name_route_to_pose_,
            std::bind(&Center::route_to_pose_goal_handle, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&Center::route_to_pose_cancel_handle, this, std::placeholders::_1),
            std::bind(&Center::route_to_pose_accepted_handle, this, std::placeholders::_1),
            rcl_action_server_get_default_options(), cbg_action_server_
    );

    //==
    // subscribe
    //==
    // imu callback

/*    cbg_imu_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_imu_options;
    sub_imu_options.callback_group = cbg_imu_;
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(constants_->tp_name_imu_, default_qos,
                                                                std::bind(&Center::imu_callback, this,
                                                                          std::placeholders::_1), sub_imu_options);*/
    // gps callback

    cbg_odom_euler_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_odom_euler_options;
    sub_odom_euler_options.callback_group = cbg_odom_euler_;
    sub_odom_eular_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(constants_->tp_name_odom_eular_, default_qos,
                                                                      std::bind(&Center::odom_eular_callback, this,
                                                                                std::placeholders::_1), sub_odom_euler_options);


    cbg_gps_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_gps_options;
    sub_gps_options.callback_group = cbg_gps_;
    sub_gps_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(constants_->tp_name_gps_, default_qos,
                                                                      std::bind(&Center::gps_callback, this,
                                                                                std::placeholders::_1),
                                                                      sub_gps_options);

    // odom callback

    cbg_odom_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_odom_options;
    sub_odom_options.callback_group = cbg_odom_;
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(constants_->tp_name_odom_, default_qos,
                                                                   std::bind(&Center::odom_callback, this,
                                                                             std::placeholders::_1),
                                                                   sub_odom_options);
    // route_deviation
    // 경로 이탈

    cbg_route_deviation_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_route_deviation_options;
    sub_route_deviation_options.callback_group = cbg_route_deviation_;
    sub_route_deviation_ = this->create_subscription<routedevation_msgs::msg::Status>(
            constants_->tp_name_route_deviation_,
            default_qos,
            std::bind(&Center::route_deviation_callback,
                      this,
                      std::placeholders::_1),
            sub_route_deviation_options);

    // 장애물 정보
    // obstacle_status

    cbg_obstacle_status_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_obstacle_status_options;
    sub_obstacle_status_options.callback_group = cbg_obstacle_status_;
    sub_obstacle_status_ = this->create_subscription<obstacle_msgs::msg::Status>(
            constants_->tp_name_obstacle_status_,
            default_qos,
            std::bind(&Center::obstacle_status_callback,
                      this,
                      std::placeholders::_1),
            sub_obstacle_status_options);

    // 속도 정보
    cbg_velocity_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_velocity_options;
    sub_velocity_options.callback_group = cbg_velocity_;
    sub_velocity_status_ = this->create_subscription<robot_status_msgs::msg::VelocityStatus>(
            constants_->tp_name_drive_velocity_,
            default_qos,
            std::bind(&Center::velocity_status_callback, this, std::placeholders::_1),
            sub_velocity_options);

    //==
    // publisher
    //==
    // cmd_vel (이동 정보)
    cbg_cmd_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions pub_cmd_options;
    pub_cmd_options.callback_group = cbg_cmd_;
    pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>(constants_->tp_name_cmd_, default_qos,
                                                                 pub_cmd_options);
    // 브레이크 작동
    // 필요시 callback group 이용할 것.
    rclcpp::PublisherOptions pub_break_options;
    pub_break_options.callback_group = cbg_break_;
    pub_break_ = this->create_publisher<route_msgs::msg::DriveBreak>(constants_->tp_name_drive_break_,
                                                                     default_qos, pub_break_options);
    rclcpp::PublisherOptions pub_drive_info_options;
    pub_drive_info_options.callback_group = cbg_drive_info_pub_;
    // drive state - 스피커, 주행 모드
    pub_drive_state_ = this->create_publisher<route_msgs::msg::DriveState>(
            constants_->tp_name_drive_info_,
            1,pub_drive_info_options);

    rclcpp::PublisherOptions pub_body_options;
    pub_body_options.callback_group = cbg_pub_body_;
    pub_body_ = this->create_publisher<can_msgs::msg::AdControlBody>(constants_->tp_name_control_body_,default_qos,pub_body_options);

    // 로봇 모드 timer
    //   ㄴ speaker timer
    // 0.1 sec = 100ms
    cbg_drive_info_timer_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);
    timer_drive_state_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                                 std::bind(&Center::drive_info_timer,this), cbg_drive_info_timer_);
#if DEBUG_MODE == 2
    RCLCPP_INFO(this->get_logger(), "setting end");
#endif
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
#if DEBUG_MODE == 1
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::cout << "[Time] " << std::ctime(&now_c) << "[Center]-[route_to_pose_goal_handle] goal start_kind : "
              << goal->start_node.kind << "end_kind : " << goal->end_node.kind << "[id] "<<goal->start_node.node_id<< std::endl;
#endif
    // 1) goal 수신
    // 2) route_to_pose_goal_handle 호출
    DataTypeTrans trans;
    car_->set_direction(trans.car_direction_determine(goal->start_node.direction));
    task_ = std::make_unique<TaskGoal>(goal->start_node, goal->end_node);
    task_->bypass_cur_node_ = goal->start_node;
    task_->bypass_next_node_ = goal->end_node;
    // * [Exception Handling] 연결 노드 일때 45도 이상 전환하지 못하도록
    try {
        CarBehavior car_behavior;
        //4) 직진 명령인가?
        if (car_behavior.straight_judgment(task_->get_cur_node_kind(), task_->get_next_node_kind())) {
            // abs(출발지 노드 진출 방향-기존 노드 진출 방향) >45
            // 4-Y) 각도가 예외 상황인가?
            // 4-Y-Y) REJECT
            // 4-Y-N) route_to_pose_accepted_handle

            if(car_behavior.car_rotation_judgment(car_->get_degree(),task_->get_cur_heading(),45)==false){
                now = std::chrono::system_clock::now();
                std::time_t now_c = std::chrono::system_clock::to_time_t(now);
                std::cout << "[Time] " << std::ctime(&now_c) << "[Center]-[route_to_pose_goal_handle]-[REJECT]-[Heading Over]"<< std::endl;
                return rclcpp_action::GoalResponse::REJECT;
            }
            else{
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            }
        }
    } //try
    catch (std::out_of_range &error) {
        // 3) 올바른 이동 명령인가?
        now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        std::cout << "[Time] " << std::ctime(&now_c) << "[Center]-[route_to_pose_goal_handle]-[REJECT]-[Goal Error]"<< std::endl;
        return rclcpp_action::GoalResponse::REJECT;
    } // catch
    // 4-N route_to_pose_accepted_handle
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
Center::route_to_pose_cancel_handle(const std::shared_ptr<RouteToPoseGoalHandler> goal_handle) {
#if DEBUG_MODE == 1
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::cout << "[Time] " << std::ctime(&now_c) << "[Center]-[route_to_pose_cancel_handle] Cancel" << std::endl;
#endif
    return rclcpp_action::CancelResponse::ACCEPT;
}

/**
 * @brief A function that operates route_to_pose_execute as a thread, responsible for the core functionality of handling goals
 * @param goal_handle
 */
void Center::route_to_pose_accepted_handle(const std::shared_ptr<RouteToPoseGoalHandler> goal_handle) {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    // 5) route_to_pose_execute
#if DEBUG_MODE == 1
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::cout << "[Time] " << std::ctime(&now_c) << "[Center]-[route_to_pose_accepted_handle] accepted" << std::endl;
#endif
    std::thread{std::bind(&Center::route_to_pose_execute, this, std::placeholders::_1), goal_handle}.detach();
}

void Center::route_to_pose_execute(const std::shared_ptr<RouteToPoseGoalHandler> goal_handle) {
#if DEBUG_MODE == 1
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::cout << "[Time] " << std::ctime(&now_c) << "[Center]-[route_to_pose_execute] execute" <<std::endl;
#endif
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    // max speed

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<RouteToPose::Feedback>();
    auto result = std::make_shared<RouteToPose::Result>();
    std::unique_ptr<Distance> center_distance = std::make_unique<Distance>();
    CarBehavior car_behavior;
    // [Cancel]
    //5-1) 취소 명령이 있는가?
    if (cancel_check(result, goal_handle)) {
        return;
    }

    /*
    // [Exception Handling]
    //5-2) 센서 데이터가 들어오는가?
    if (car_->get_location().fn_get_longitude() == 0 && car_->get_location().fn_get_latitude() == 0) {
            feedback->status_code = static_cast<int>(kec_driving_code::FeedBack::kSensorWaiting);
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Publish feedback");
    }
    */

    double car_latitude = car_->get_location().fn_get_latitude();
    double car_longitude = car_->get_location().fn_get_longitude();
    GpsData cur_location;
    cur_location.fn_set_latitude(car_latitude);
    cur_location.fn_set_longitude(car_longitude);
    init_distance = center_distance->distance_from_perpendicular_line(task_->get_cur_gps(),
                                                                             task_->get_next_gps(),
                                                                             cur_location)+3;
    //6-1) 진진 주행
    if (car_behavior.straight_judgment(task_->get_cur_node_kind(), task_->get_next_node_kind())) {
        straight_move(feedback, result, goal_handle, car_behavior);
    } // 직진 주행
        //6-2) 회전 주행
    else if (car_behavior.intersection_judgment(task_->get_cur_node_kind(),
                                                task_->get_next_node_kind())&&car_behavior.waiting_judgment(task_->get_cur_node_kind())==false) {
        turn_move(feedback, result, goal_handle, car_behavior);
    } // 회전 주행

        // 대기
    else if (car_behavior.waiting_judgment(task_->get_cur_node_kind())) {
        while (rclcpp::ok()) {
            // stop
#if DEBUG_MODE ==1
            RCLCPP_INFO(this->get_logger(), "[waiting mode] start");
#endif
            if(waiting_check_==false) {
                geometry_msgs::msg::Twist stop;
                stop.angular.x = 0;
                stop.linear.z = 0;
                pub_cmd_->publish(stop);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                // 교차로 정보 확인
                car_->set_drive_mode(kec_car::DrivingMode::kStop);
            }
            if (obs_status_->obstacle_status != 0 && obs_status_->obstacle_value==false) {
                waiting_check_=true;
                car_->set_drive_mode(kec_car::DrivingMode::kStraight);
                break;
            } else {
                continue;
            }
        }
        if (task_->get_next_node_kind() == kec_car::NodeKind::kIntersection) {
            turn_move(feedback, result, goal_handle, car_behavior);
            car_->set_drive_mode(kec_car::DrivingMode::kCrossroads);
        } else {
            straight_move(feedback, result, goal_handle, car_behavior);
            car_->set_drive_mode(kec_car::DrivingMode::kStraight);
        }

    }// 대기


    // 7) 완료 처리
    if (rclcpp::ok()) {
        result->result = static_cast<int>(kec_driving_code::Result::kSuccess);
        goal_handle->succeed(result);
        feedback_check_ = false;
        waiting_check_=false;
        car_->set_drive_mode(kec_car::DrivingMode::kArrive);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        std::this_thread::sleep_for(std::chrono::seconds (1));
    }
}

/**
 * @brief imu callback, 차량 각도 제공
 * @param imu
 */
void Center::imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu) {
#if DEBUG_MODE == 2
    std::cout <<"[Center]-[imu_callback] : "<<
            "velocity : " << imu->angular_velocity.x << '\n' <<
              "frame_id : " << imu->header.frame_id << '\n' <<
              "orientation : " << imu->orientation.x << std::endl;
#endif
    std::lock_guard<std::mutex> lock(mutex_);
    imu_converter_->set_correction(ros_parameter_->imu_correction_);
    car_->set_degree(static_cast<float>(imu_converter_->quaternion_to_heading_converter(imu)));
}

void Center::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr gps) {
#if DEBUG_MODE == 2
    std::cout << "[Center]-[gps_callback] : " <<
              "latitude : " << gps->latitude << '\n' <<
              "longitude : " << gps->longitude << std::endl;
#endif
    std::lock_guard<std::mutex> lock(mutex_);
    GpsData data(gps->latitude, gps->longitude);
    car_->set_location(data);
}

/**
 * @brief 테스트 후 수정 예정
 */
void Center::ros_parameter_setting() {
    this->declare_parameter<float>("imu_correction", SETTING_ZERO);
    this->declare_parameter<float>("max_speed", SETTING_ZERO);
    this->declare_parameter<float>("driving_calibration_max_angle", SETTING_ZERO);
    this->declare_parameter<float>("driving_calibration_min_angle", SETTING_ZERO);
    this->declare_parameter<float>("driving_calibration_angle_increase", SETTING_ZERO);
    this->declare_parameter<float>("goal_distance", SETTING_ZERO);
    this->declare_parameter<float>("rotation_straight_dist", SETTING_ZERO);
    this->declare_parameter<float>("rotation_angle_increase", SETTING_ZERO);
    this->declare_parameter<float>("rotation_angle_tolerance", SETTING_ZERO);
    this->declare_parameter<float>("recovery_goal_tolerance", SETTING_ZERO);
    this->declare_parameter<float>("deceleration", SETTING_ZERO);
    this->declare_parameter<float>("friction_coefficient", SETTING_ZERO);
    float imu_correction;
    float max_speed;
    float driving_calibration_min_angle;
    float driving_calibration_max_angle;
    float driving_calibration_angle_increase;
    float goal_distance;
    float rotation_straight_dist;
    float rotation_angle_increase;
    float rotation_angle_tolerance;
    float recovery_goal_tolerance;
    float deceleration;
    float friction_coefficient;
    this->get_parameter("imu_correction", imu_correction);
    this->get_parameter("max_speed", max_speed);
    this->get_parameter("driving_calibration_min_angle", driving_calibration_min_angle);
    this->get_parameter("driving_calibration_max_angle", driving_calibration_max_angle);
    this->get_parameter("driving_calibration_angle_increase", driving_calibration_angle_increase);
    this->get_parameter("goal_distance", goal_distance);
    this->get_parameter("rotation_straight_dist", rotation_straight_dist);
    this->get_parameter("rotation_angle_increase", rotation_angle_increase);
    this->get_parameter("rotation_angle_tolerance", rotation_angle_tolerance);
    this->get_parameter("recovery_goal_tolerance", recovery_goal_tolerance);
    this->get_parameter("deceleration", deceleration);
    this->get_parameter("friction_coefficient", friction_coefficient);
    ros_parameter_ = std::make_unique<RosParameter>(imu_correction,
                                                    max_speed,
                                                    driving_calibration_max_angle,
                                                    driving_calibration_min_angle,
                                                    driving_calibration_angle_increase,
                                                    goal_distance,
                                                    rotation_straight_dist,
                                                    rotation_angle_increase,
                                                    rotation_angle_tolerance,
                                                    recovery_goal_tolerance,
                                                    deceleration,
                                                    friction_coefficient);
}


void Center::calculate_straight_movement(float acceleration) {
    if(car_->get_direction()==kec_car::Direction::kBackward){
        acceleration=-acceleration;
    }
    geometry_msgs::msg::Twist result;
    result.linear.x=(acceleration);
    result.linear.y=(SETTING_ZERO);
    result.linear.z=(SETTING_ZERO);
    // 현재 각도-노드진입 탈출 각도를 통하여 링크와 얼마나 다른 방향으로 가는지 확인

    double car_degree = car_->get_degree();
    double node_degree = task_->get_cur_heading();
    if(car_degree>=360){
        car_degree=car_degree-360;
    }
    else if(node_degree>=360){
        node_degree = node_degree-360;
    }

    if(car_degree>270&& node_degree<90){
        car_degree= 360-car_degree;
    }
    else if(car_degree<90 && node_degree>270){
        car_degree = 270+car_degree;
    }

    const double link_degree =  car_degree-node_degree;
    // 각도에 따라 더 틀지 결정
    const double first_zone = ros_parameter_->driving_calibration_max_angle_ * FIRST_ZONE;
    const double second_zone = ros_parameter_->driving_calibration_max_angle_ * SECOND_ZONE;
    // 왼쪽/ 오른쪽
    const double direction = (link_degree > 0) ? ros_parameter_->driving_calibration_angle_increase_
                                               : -ros_parameter_->driving_calibration_angle_increase_;
    if (link_degree > ros_parameter_->driving_calibration_min_angle_) {
        if (link_degree < first_zone) {
            result.angular.z=direction;
        } else if (link_degree > first_zone &&
                   link_degree < second_zone) {
            result.angular.z=direction * 2;
        } else if (link_degree > second_zone) {
            result.angular.z=direction * 3;
        }
    } else {
        //직선 주행
/*        result.angular.x = (acceleration);
        result.angular.y = (SETTING_ZERO);
        result.angular.z = (SETTING_ZERO);*/
    }
    pub_cmd_->publish(result);
}

void Center::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom) {
#if DEBUG_MODE == 2
    std::cout <<"[Center]-[odom_callback] : "<<
              "position : " << odom->pose.pose.position.x << '\n' <<
              "orientation : " << odom->pose.pose.orientation.x << '\n' <<
              "frame_id : " << odom->header.frame_id << std::endl;
#endif
}

void Center::route_deviation_callback(const routedevation_msgs::msg::Status::SharedPtr status) {
#if DEBUG_MODE == 1
    std::cout <<"[Center]-[route_deviation_callback] : "<<
              "offcource_status : " << status->offcource_status << '\n' <<std::endl;
#endif
    // 복구모드 시 갱신 필요 없음.
    // 복구 중 또 들어오는 경우를 방지하기 위함.
    if (car_->get_drive_mode() == kec_car::DrivingMode::kRecovery) {
        return;
    }
    devation_status_ = status;
}

/**
 * @brief obstacle_status : 0 - drive, 1 - intersection, 2 - parking
 * @brief obstacle_value :
 * @param status
 */
void Center::obstacle_status_callback(const obstacle_msgs::msg::Status::SharedPtr status) {
#if DEBUG_MODE == 1
    std::cout <<"[Center]-[obstacle_status_callback] : "<<
              "obstacle_value : " << status->obstacle_value << '\n' <<
              "obstacle_status : " << static_cast<int>(status->obstacle_status) << std::endl;
#endif
    std::unique_ptr<Distance> center_distance = std::make_unique<Distance>();
    route_msgs::msg::DriveBreak temp_break;
    obs_status_ = status;
    if (obs_status_->obstacle_status == 0) {
        if (obs_status_->obstacle_value==true) {
            if (obs_status_->obstacle_distance > center_distance->distance_braking_calculate(
                    car_->get_speed(),
                    car_->get_friction_coefficient(),
                    car_->get_deceleration())) {
                // 속도 감속이 필요한 상황
                temp_break.break_pressure = 70;
                pub_break_->publish(temp_break);
            } else {
                // 정지가 필요한 상황
                temp_break.break_pressure = 100;
                pub_break_->publish(temp_break);
                geometry_msgs::msg::Twist stop_twist ;
                stop_twist.linear.x =0;
                stop_twist.angular.z=0;
                pub_cmd_->publish(stop_twist);
            }
        } else {
            temp_break.break_pressure = 0;
            pub_break_->publish(temp_break);
        }
    }
}

void Center::drive_info_timer() {
    DataTypeTrans data_type_trans;
    route_msgs::msg::DriveState drive_state;
    drive_state.code = data_type_trans.drive_mode_to_string(car_->get_drive_mode());
    if (car_->get_drive_mode() == kec_car::DrivingMode::kStraight) {
        drive_state.speaker = 1003;
    } else if (car_->get_drive_mode() == kec_car::DrivingMode::kParking ||
               car_->get_drive_mode() == kec_car::DrivingMode::kCrossroads) {
        drive_state.speaker = 2002;
    }
    if(task_!= nullptr) {
        drive_state.start_node= task_->bypass_cur_node_;
        drive_state.end_node= task_->bypass_next_node_;
    }
    pub_drive_state_->publish(drive_state);
}

void Center::start_on(const std::shared_ptr<RouteToPose::Feedback> feedback,
                      const std::shared_ptr<RouteToPoseGoalHandler> goal_handle) {
    feedback->status_code = static_cast<int>(kec_driving_code::FeedBack::kStart);
    if(!feedback_check_) {
        goal_handle->publish_feedback(feedback);
        feedback_check_=true;
    }
    //RCLCPP_INFO(this->get_logger(), "Publish feedback");

}

bool Center::cancel_check(const std::shared_ptr<RouteToPose::Result> result,
                          const std::shared_ptr<RouteToPoseGoalHandler> goal_handle) {
    if (goal_handle->is_canceling()) {
        result->result = static_cast<int>(kec_driving_code::Result::kFailedCancel);
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return true;
    }
    return false;
}

void Center::car_rotation(CarBehavior car_behavior, double node_heading, kec_car::NodeKind node_kind) {
    double point_node_heading = node_heading;
    double car_heading = car_->get_degree();
    //float car_speed = car_->get_speed();
    float car_speed = 0.8;


    if (car_behavior.car_move_direct(car_heading,
                                     point_node_heading)) {
        //right
        geometry_msgs::msg::Twist cmd_vel = car_behavior.calculate_rotation_movement(
                car_speed, -ros_parameter_->driving_calibration_max_angle_);
#if DEBUG_MODE ==1
        RCLCPP_INFO(this->get_logger(), "[car_rotation] linear.x %f angular.z %f car_ %f node_ %f",
                    cmd_vel.linear.x,cmd_vel.angular.z, car_->get_degree(), node_heading);
#endif
        pub_cmd_->publish(cmd_vel);
        if(node_kind==kec_car::NodeKind::kIntersection){
            can_msgs::msg::AdControlBody temp_body;
            temp_body.right_turn_light=true;
            temp_body.brake_light=false;
            pub_body_->publish(temp_body);
        }
    } else {
        //left
        geometry_msgs::msg::Twist cmd_vel = car_behavior.calculate_rotation_movement(
                car_speed, ros_parameter_->driving_calibration_max_angle_);
#if DEBUG_MODE ==1
        RCLCPP_INFO(this->get_logger(), "[car_rotation] linear.x %f angular.z %f car_ %f node_ %f",
                    cmd_vel.linear.x,cmd_vel.angular.z, car_->get_degree(), node_heading);
#endif
        pub_cmd_->publish(cmd_vel);
        if(node_kind==kec_car::NodeKind::kIntersection){
            can_msgs::msg::AdControlBody temp_body;
            temp_body.left_turn_light=true;
            temp_body.brake_light=false;
            pub_body_->publish(temp_body);
        }
    }
}

void Center::straight_move(const std::shared_ptr<RouteToPose::Feedback> feedback,
                           const std::shared_ptr<RouteToPose::Result> result,
                           const std::shared_ptr<RouteToPoseGoalHandler> goal_handle,
                           CarBehavior car_behavior) {

    if (task_->get_cur_node_kind() == kec_car::NodeKind::kEndpoint) {
        car_->set_drive_mode(kec_car::DrivingMode::kParking);
    } else {
        car_->set_drive_mode(kec_car::DrivingMode::kStraight);
    }

    std::unique_ptr<Distance> center_distance = std::make_unique<Distance>();
    // 직진 주행을 위한 반복문
    while (rclcpp::ok()) {
        if (cancel_check(result, goal_handle)) {
            return;
        }
        // 도착지 거리
        // 6-1-1) 목적지 도착 여부
        // 파라미터 작업
        double car_latitude = car_->get_location().fn_get_latitude();
        double car_longitude = car_->get_location().fn_get_longitude();
        GpsData cur_location;
        cur_location.fn_set_latitude(car_latitude);
        cur_location.fn_set_longitude(car_longitude);
        double goal_distance = center_distance->distance_from_perpendicular_line(task_->get_cur_gps(),
                                                                                 task_->get_next_gps(),
                                                                                 cur_location);
#if DEBUG_MODE == 1
        RCLCPP_INFO(this->get_logger(), "[distance] %f", goal_distance);
#endif
        if (goal_distance <= ros_parameter_->goal_distance_ || goal_distance>init_distance) {
#if DEBUG_MODE == 2
            RCLCPP_INFO(this->get_logger(), "[goal]");
#endif
            car_->set_drive_mode(kec_car::DrivingMode::kArrive);
            route_msgs::msg::DriveBreak drive_break;
            drive_break.break_pressure = 100;
            pub_break_->publish(drive_break);
            geometry_msgs::msg::Twist result_vel;
            result_vel.linear.x = 0;
            result_vel.angular.z = 0;
            pub_cmd_->publish(result_vel);
            break;
        }
        // 6-1-2) 이탈 정보 수신 -- route_deviation_callback
        // 6-1-3) 이탈 되었는가?
        mutex_.lock();
        routedevation_msgs::msg::Status temp_devation_status = *devation_status_;
        mutex_.unlock();
        // 6-1-3-Y)
        if (temp_devation_status.offcource_status) {
#if DEBUG_MODE ==1
            RCLCPP_INFO(this->get_logger(), "[recovery mode] start");
#endif
            car_->set_drive_mode(kec_car::DrivingMode::kRecovery);
            // 자동차 자세 - 출발지 노드 헤딩 정보 >45 실패
            // 자동차 자세 -
            if (car_behavior.car_rotation_judgment(car_->get_degree(),task_->get_cur_heading(),45)== false) {
                if (rclcpp::ok()) {
                    result->result = static_cast<int>(kec_driving_code::Result::kFailedErrorRoute);
                    goal_handle->abort(result);
                    RCLCPP_INFO(this->get_logger(), "Goal Failed");
                    return;
                }
            } else {
#if DEBUG_MODE ==1
                RCLCPP_INFO(this->get_logger(), "[recovery mode] goal check success");
#endif
                car_->set_drive_mode(kec_car::DrivingMode::kRecovery);
                // 6-1-4) 복구 목적지 설정
                GpsData gps_data(devation_status_->offcource_goal_lat,
                                 devation_status_->offcource_goal_lon);
                while (rclcpp::ok()) {
                    if (cancel_check(result, goal_handle)) {
                        return;
                    }
                    // 6-1-5) 복귀 목적지 도착 여부 N
                    double recovery_goal_distance = center_distance->distance_from_perpendicular_line(
                            task_->get_cur_gps(), gps_data,
                            car_->get_location());
#if DEBUG_MODE ==1
                    RCLCPP_INFO(this->get_logger(), "[recovery mode] goal distance : %f",recovery_goal_distance);
#endif
                    if (ros_parameter_->recovery_goal_tolerance_ <recovery_goal_distance) {
                        // 6-1-6) 각도 변경 필요? Y
                        GpsData temp_car_degree = car_->get_location();
                        GpsData temp_goal_degree = task_->get_next_gps();
                        double goal_angle = center_distance->calculate_line_angle(temp_car_degree,temp_goal_degree);
#if DEBUG_MODE ==2
                        // 확인 후 지울 것.
           /*             RCLCPP_INFO(this->get_logger(), "[recovery mode] temp_car_degree : %lf temp_goal_degree : %lf goal_angle : %lf ",
                                    temp_car_degree,
                                    temp_goal_degree,
                                    goal_angle);*/
#endif
                        if (car_behavior.car_rotation_judgment(
                                car_->get_degree(),
                                goal_angle,
                                ros_parameter_->rotation_angle_tolerance_)==false) {
                            // 6-1-7) 휠제어
                            car_rotation(car_behavior,
                                         task_->get_cur_heading(),task_->get_next_node_kind());
                        }
                            // 6-1-6) 각도 변경 필요? N
                        else {
                            // 6-1-7) 직진
                            calculate_straight_movement(
                                    ros_parameter_->max_speed_);
                        }
                    }// 6-1-5) 복귀 목적지 도착 여부 N
                    else {
#if DEBUG_MODE ==1
                        RCLCPP_INFO(this->get_logger(), "[recovery mode] goal");
#endif
                        car_->set_drive_mode(kec_car::DrivingMode::kStop);
                        break;
                    }// 6-1-5) 복귀 목적지 도착 여부 Y
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                } // 복귀 주행 반복문
            }
        }// 6-1-3-N) 이탈 되었는가?
        else {
            //직진 주행 추가
            //calculate_straight_movement(0.2);
            calculate_straight_movement(ros_parameter_->max_speed_);
            car_->set_drive_mode(kec_car::DrivingMode::kStraight);
            //feedback publish
            start_on(feedback, goal_handle);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

void Center::turn_move(const std::shared_ptr<RouteToPose::Feedback> feedback,
                       const std::shared_ptr<RouteToPose::Result> result,
                       const std::shared_ptr<RouteToPoseGoalHandler> goal_handle,
                       CarBehavior car_behavior) {
#if DEBUG_MODE ==1
    RCLCPP_INFO(this->get_logger(), "[turn_move mode] start");
#endif
    std::unique_ptr<Distance> center_distance = std::make_unique<Distance>();
    double init_distance = center_distance->distance_from_perpendicular_line(
            task_->get_cur_gps(), task_->get_next_gps(), car_->get_location());
    while (rclcpp::ok()) {
        // 2-5) 장애물 존재 여부 확인
        if (obs_status_->obstacle_value==true && obs_status_->obstacle_status==0) {
            //obstacle_value = ture - 감지, false- 미감지
            continue;
        } else {
            if (task_->get_cur_node_kind() == kec_car::NodeKind::kIntersection) {
                car_->set_drive_mode(kec_car::DrivingMode::kCrossroads);
            }
            double goal_distance = center_distance->distance_from_perpendicular_line(
                    task_->get_cur_gps(), task_->get_next_gps(), car_->get_location());
            // 회전 주행 알림.
            car_->set_drive_mode(kec_car::DrivingMode::kCrossroads);
            //feedback publish
            start_on(feedback, goal_handle);

#if DEBUG_MODE ==1
            RCLCPP_INFO(this->get_logger(), "[turn_move mode] - straight goal check goal_distance %f init %f rotation_straight_dist_ %f "
            ,goal_distance,init_distance,ros_parameter_->rotation_straight_dist_);
#endif
            // 2-6) 직진 목적지에 도착했는가?
            // 회전 중 틀어지지 않도록 task_->rotation.. 을 통해 목적지 판단 무시
            // 최초에는 목적지 도착하여 if문, 이후에는 check를 통해 진입
            if (goal_distance < (init_distance-ros_parameter_->rotation_straight_dist_) ||
                (task_->rotation_straight_check_)) {
                // 2-6-1) 도착함
                // 2-7) 교차로 노드 헤딩 정보로 방향 확인
                task_->rotation_straight_check_ = true;
                double next_node_heading = task_->get_next_heading();
                double car_heading = car_->get_degree();
                // 2-8) 회전 여부 확인
                if (car_behavior.car_rotation_judgment(
                        car_heading,
                        next_node_heading,
                        ros_parameter_->rotation_angle_tolerance_)==false) {
                    car_rotation(car_behavior, next_node_heading,task_->get_next_node_kind());
                } else {
                    // 2-9) 종료
                    car_->set_drive_mode(kec_car::DrivingMode::kStop);
                    geometry_msgs::msg::Twist turn_success;
                    turn_success.linear.x=0;
                    turn_success.angular.z=0;
                    pub_cmd_->publish(turn_success);
                    task_->rotation_straight_check_ = false;
                    break;
                }

            } else {
                //2-6-2) 도착하지 않음
#if DEBUG_MODE ==1
                RCLCPP_INFO(this->get_logger(), "[turn_move mode]-[straight] start");
#endif

                calculate_straight_movement(ros_parameter_->max_speed_);
            } // 2-6-2)
        }// 2-5)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void Center::velocity_status_callback(const robot_status_msgs::msg::VelocityStatus::SharedPtr status) {
    car_->set_speed(status->current_velocity);
}

void Center::timer_callback() {
    RCLCPP_INFO(this->get_logger(), "Sending request");
}

void Center::odom_eular_callback(const geometry_msgs::msg::PoseStamped::SharedPtr odom_eular) {
    car_->set_degree(odom_eular->pose.orientation.y);
}