//
// Created by nuc-bt on 24. 3. 5.
//

#include <utility>

#include "route_tracker/center.hpp"
#include "code/kec_driving_data_code.hpp"
#include "distance.hpp"
#include "common/data_type_trans.hpp"

#include "common/test.h"

#define FIRST_ZONE 0.3
#define SECOND_ZONE 0.6
#define SETTING_ZERO 0.0
#define MAX_DIRECTION 45
#define DRIVING_INFO_HZ 100
using namespace std::chrono_literals;

Center::Center() : Node("route_tracker_node"), feedback_check_(false), waiting_check_(false),obstacle_first_check_(false),speaker_seq_(0),prev_speed_(0),init_distance(0){
    RCLCPP_INFO(this->get_logger(),
                R"(
 ██████  ██████  ███████         ███    ██  █████  ██    ██ ██
██       ██   ██ ██              ████   ██ ██   ██ ██    ██ ██
██   ███ ██████  ███████         ██ ██  ██ ███████ ██    ██ ██
██    ██ ██           ██         ██  ██ ██ ██   ██  ██  ██  ██
 ██████  ██      ███████ ███████ ██   ████ ██   ██   ████   ██)");
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
    obstacle_msgs::msg::Status init_obstacle;
    init_obstacle.obstacle_status = 0;
    init_obstacle.obstacle_value = false;
    //
    routedevation_msgs::msg::Status init_routedevation;
    init_routedevation.offcource_status = false;
    //
    GpsData data(0, 0);

    car_->set_location(data);
    car_->set_degree(0);
    car_->set_drive_mode(kec_car::DrivingMode::kStop);
    obs_status_ = std::make_shared<obstacle_msgs::msg::Status>(init_obstacle);
    prev_status_= std::make_shared<obstacle_msgs::msg::Status>(init_obstacle);
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
    sub_odom_eular_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(constants_->tp_name_odom_eular_,
                                                                                 default_qos,
                                                                                 std::bind(&Center::odom_eular_callback,
                                                                                           this,
                                                                                           std::placeholders::_1),
                                                                                 sub_odom_euler_options);


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
            1, pub_drive_info_options);

    rclcpp::PublisherOptions pub_body_options;
    pub_body_options.callback_group = cbg_pub_body_;
    pub_body_ = this->create_publisher<can_msgs::msg::AdControlBody>(constants_->tp_name_control_body_, default_qos,
                                                                     pub_body_options);

    rclcpp::PublisherOptions pub_obs_event_options;
    pub_obs_event_options.callback_group=cbg_pub_obs_event_;
    pub_obs_event_ = this->create_publisher<obstacle_msgs::msg::Status>(constants_->tp_name_obstacle_event_,default_qos,
                                                                        pub_obs_event_options);

    rclcpp::PublisherOptions pub_imu_offest_options;
    pub_imu_offest_options.callback_group = cbg_pub_imu_offset_;
    pub_imu_offset_ = this->create_publisher<route_msgs::msg::Offset>(constants_->tp_name_imu_offset_,default_qos,
                                                                       pub_imu_offest_options);

    rclcpp::PublisherOptions pub_log_options;
    pub_log_options.callback_group =cbg_pub_log_;
    pub_log_ = this->create_publisher<route_msgs::msg::Log>("route_tracker_log",default_qos,
                                                               pub_log_options);

    // 로봇 모드 timer
    //   ㄴ speaker timer
    // 0.1 sec = 100ms
    cbg_drive_info_timer_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);
    timer_drive_state_ = this->create_wall_timer(std::chrono::milliseconds(DRIVING_INFO_HZ),
                                                 std::bind(&Center::drive_info_timer, this), cbg_drive_info_timer_);
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
    RCLCPP_INFO(this->get_logger(),
                "[Center]-[route_to_pose_goal_handle]-[Fist Goal Start]\n"
                "[Start id] %s [kind] %s [driving_option] %s [direction] %s [lat] %f [long] %f [NodeKind] %s [Heading] %lf\n"
                "[End id] %s [kind] %s [driving_option] %s [direction] %s [lat] %f [long] %f [NodeKind] %s [Heading] %lf",
                goal->start_node.node_id.c_str(),
                goal->start_node.kind.c_str(),
                goal->start_node.driving_option.c_str(),
                goal->start_node.direction.c_str(),
                goal->start_node.position.latitude,
                goal->start_node.position.longitude,
                goal->start_node.kind.c_str(),
                goal->start_node.heading,
                goal->end_node.node_id.c_str(),
                goal->end_node.kind.c_str(),
                goal->end_node.driving_option.c_str(),
                goal->end_node.direction.c_str(),
                goal->end_node.position.latitude,
                goal->end_node.position.longitude,
                goal->end_node.kind.c_str(),
                goal->end_node.heading);
#endif
    cancel_check_= false;
    reject_check_= false;
    failed_check_= false;
    // 1) goal 수신
    // 2) route_to_pose_goal_handle 호출
    try {
        DataTypeTrans trans;
        car_->set_direction(trans.car_direction_determine(goal->start_node.direction));
        // goal->start_node 변경
        // node 직접 계산

        Distance distance;
        GpsData start(goal->start_node.position.latitude,goal->start_node.position.longitude);
        GpsData end(goal->end_node.position.latitude,goal->end_node.position.longitude);

        task_ = std::make_unique<TaskGoal>(goal->start_node, goal->end_node);
        task_->bypass_cur_node_ = goal->start_node;
        task_->bypass_next_node_ = goal->end_node;

        // imu offset 설정
        DegreeConvert degree_convert;
        route_msgs::msg::Offset imu_offset;
        auto [degrees, fraction] = degree_convert.parse_input(static_cast<float>(task_->get_cur_heading()));

#if DEBUG_MODE == 1
        RCLCPP_INFO(this->get_logger(),
                    "[Center]-[route_to_pose_goal_handle]-[imu-offset-setting] %d - %lf\n",degrees,fraction);
#endif
        imu_offset.data = static_cast<float>(fraction);
        imu_offset.stamp = this->now();
        pub_imu_offset_->publish(imu_offset);

        // EndNode Setting
        if(task_->get_next_node_kind()==kec_car::NodeKind::kEndpoint){
            task_->set_cur_degree(static_cast<float>(degrees));
        }
        else {
            task_->set_cur_degree(static_cast<float>(distance.calculate_line_angle(start, end)));
        }
        if(task_->get_cur_dir()==kec_car::Direction::kBackward){
            CarBehavior car_behavior;
            double reverse_degree = car_behavior.car_degree_reverse(task_->get_cur_heading());
            task_->set_cur_degree(static_cast<float>(reverse_degree));
        }

        if(task_->get_next_node_kind()==kec_car::NodeKind::kIntersection){
                DegreeConvert next_dc;
                route_msgs::msg::Offset next_imu_offset;
                auto [ndegrees, nfraction] = next_dc.parse_input(static_cast<float>(task_->get_next_heading()));
                next_imu_offset.data = static_cast<float>(nfraction);
                next_imu_offset.stamp = this->now();
                task_->set_next_degree(static_cast<float>(ndegrees));
                pub_imu_offset_->publish(next_imu_offset);
        }

#if DEBUG_MODE == 1
        RCLCPP_INFO(this->get_logger(), "[Center]-[route_to_pose_goal_handle]-[TaskDegreeSetting]- %f",task_->get_cur_heading());
        DataTypeTrans log_trans;
        log_publish(0,
                    0,
                    car_->get_degree(),
                    task_->get_cur_heading(),
                    log_trans.drive_mode_to_string(car_->get_drive_mode()),
                    "FirstSetting");
#endif
    }
    catch(...){
        RCLCPP_INFO(this->get_logger(),"GoalHandler Error");
        reject_check_=true;
        return rclcpp_action::GoalResponse::REJECT;
    }


    // * [Exception Handling] 연결 노드 일때 45도 이상 전환하지 못하도록
    try {
        CarBehavior car_behavior;
        //4) 직진 명령인가?
        if (car_behavior.straight_judgment(task_->get_cur_node_kind(), task_->get_next_node_kind())) {
            // abs(출발지 노드 진출 방향-기존 노드 진출 방향) >45
            // 4-Y) 각도가 예외 상황인가?
            // 4-Y-Y) REJECT
            // 4-Y-N) route_to_pose_accepted_handle
            double check_car_degree = car_->get_degree();
            double check_cur_task_degree = task_->get_cur_heading();
            if (car_behavior.car_rotation_judgment(check_car_degree, check_cur_task_degree, 45) == false) {
                //now = std::chrono::system_clock::now();
                //std::time_t now_c = std::chrono::system_clock::to_time_t(now);
#if DEBUG_MODE == 1
                RCLCPP_INFO(this->get_logger(),
                            "[Center]-[route_to_pose_goal_handle]-[REJECT]-[Heading Over] car degree %lf , task cur_heading %lf",
                            check_car_degree, check_cur_task_degree);
#endif
                reject_check_ = true;
                return rclcpp_action::GoalResponse::REJECT;
            } else {
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            }
        }
    } //try
    catch (...) {
        // 3) 올바른 이동 명령인가?
#if DEBUG_MODE == 3
        RCLCPP_INFO(this->get_logger(), "[Center]-[route_to_pose_goal_handle]-[REJECT]-[Goal Error]");
#endif
        reject_check_ = true;
        return rclcpp_action::GoalResponse::REJECT;
    } // catch
    // 4-N route_to_pose_accepted_handle
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
Center::route_to_pose_cancel_handle(const std::shared_ptr<RouteToPoseGoalHandler> goal_handle) {
#if DEBUG_MODE == 1
    RCLCPP_INFO(this->get_logger(),
                "[Center]-[route_to_pose_cancel_handle]-[Cancel] %s", get_time().c_str());
#endif
(void)goal_handle;
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
    RCLCPP_INFO(this->get_logger(),
                "[Center]-[route_to_pose_accepted_handle]-[accepted] %s", get_time().c_str());
#endif
    std::thread{std::bind(&Center::route_to_pose_execute, this, std::placeholders::_1), goal_handle}.detach();
}

void Center::route_to_pose_execute(const std::shared_ptr<RouteToPoseGoalHandler> goal_handle) {
#if DEBUG_MODE == 1
    RCLCPP_INFO(this->get_logger(),
                "[Center]-[route_to_pose_execute]-[execute] %s", get_time().c_str());
#endif
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
    GpsData cur_location;
    if (task_->get_cur_driving_option() == kec_car::DrivingOption::kGps) {
        double car_latitude = car_->get_location().fn_get_latitude();
        double car_longitude = car_->get_location().fn_get_longitude();
        cur_location.fn_set_latitude(car_latitude);
        cur_location.fn_set_longitude(car_longitude);

    } else if (task_->get_cur_driving_option() == kec_car::DrivingOption::kOdom) {
        cur_location = task_->get_cur_gps();
    }
    // 점검 필요
/*    init_distance = center_distance->distance_from_perpendicular_line(task_->get_cur_gps(),
                                                                      task_->get_next_gps(),
                                                                      cur_location) + 3;*/
    init_distance = center_distance->distance_from_perpendicular_line(task_->get_cur_gps(),
                                                                      task_->get_next_gps(),
                                                                      cur_location);

#if DEBUG_MODE == 1
    RCLCPP_INFO(this->get_logger(),
                "[Center]-[route_to_pose_execute]-[INIT_SETTING] init - %lf, time : %s", init_distance,get_time().c_str());
#endif

    if (task_->get_cur_driving_option() == kec_car::DrivingOption::kGps) {
#if DEBUG_MODE == 1
        RCLCPP_INFO(this->get_logger(),
                    "[Center]-[route_to_pose_execute]-[GPS_SETTING]");
#endif
        //6-1) 진진 주행
        if (car_behavior.straight_judgment(task_->get_cur_node_kind(), task_->get_next_node_kind())) {
/*            if(task_->get_next_node_kind()==kec_car::NodeKind::kIntersection){
#if DEBUG_MODE == 1
                RCLCPP_INFO(this->get_logger(), "[route_to_pose_execute] next cross - init_dist %f",init_distance);
#endif
                init_distance = init_distance - ros_parameter_->rotation_straight_dist_;
            }*/
            straight_move(feedback, result, goal_handle, car_behavior);
        } // 직진 주행
            //6-2) 회전 주행
        else if (car_behavior.intersection_judgment(task_->get_cur_node_kind(),
                                                    task_->get_next_node_kind()) &&
                 car_behavior.waiting_judgment(task_->get_cur_node_kind()) == false) {
            turn_move(feedback, result, goal_handle, car_behavior);
        } // 회전 주행
        else if (car_behavior.waiting_judgment(task_->get_cur_node_kind())) {
            // 대기
            car_behavior.cmd_slowly_stop(pub_cmd_, pub_break_);
            while (rclcpp::ok()) {
                    if (cancel_check(result, goal_handle)) {
                        return;
                    }
                // stop
#if DEBUG_MODE == 1
                RCLCPP_INFO(this->get_logger(), "[waiting mode] start");
#endif
                if (waiting_check_ == false) {
                    car_behavior.cmd_slowly_stop(pub_cmd_, pub_break_);

                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    // 교차로 정보 확인
                    car_->set_drive_mode(kec_car::DrivingMode::kStop);
                }
                if (obs_status_->obstacle_status != 0 && obs_status_->obstacle_value == false) {
                    waiting_check_ = true;
                    brake_unlock();
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
    }
    else if(task_->get_cur_driving_option() == kec_car::DrivingOption::kOdom){
#if DEBUG_MODE == 1
        RCLCPP_INFO(this->get_logger(),
                    "[Center]-[route_to_pose_execute]-[ODOM_SETTING]");
#endif
        if(task_->get_next_node_kind()==kec_car::NodeKind::kIntersection){
            car_->set_drive_mode(kec_car::DrivingMode::kCrossroads);
            turn_move(feedback, result, goal_handle, car_behavior);
        }
        else {
            odom_move(feedback, result, goal_handle);
        }
    }

    // 7) 완료 처리
    if (rclcpp::ok()) {
        if(cancel_check_){
            cmd_stop();
            return;
        }
        else if(reject_check_){
            cmd_stop();
            return;
        }
        else if(failed_check_){
            cmd_stop();
            return;
        }
        else{
        result->result = static_cast<int>(kec_driving_code::Result::kSuccess);
        car_->set_drive_mode(kec_car::DrivingMode::kArrive);
        {
            std::unique_lock<std::mutex> ulm(goal_mutex_);
            cv_.wait_for(ulm, std::chrono::seconds(1));
        }
        goal_handle->succeed(result);
        feedback_check_ = false;
        waiting_check_ = false;
        RCLCPP_INFO(this->get_logger(), "Goal successed");
        }
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
    //std::unique_lock<std::mutex> lock(mutex_);
    //imu_converter_->set_correction(ros_parameter_->imu_correction_);
    //car_->set_degree(static_cast<float>(imu_converter_->quaternion_to_heading_converter(imu)));
}

void Center::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr gps) {
#if DEBUG_MODE == 2
    std::cout << "[Center]-[gps_callback] : " <<
              "latitude : " << gps->latitude << '\n' <<
              "longitude : " << gps->longitude << std::endl;
#endif
    std::unique_lock<std::mutex> lock(mutex_);
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
    this->declare_parameter<float>("near_destination_dist",SETTING_ZERO);
    this->declare_parameter<float>("rotation_max_speed",SETTING_ZERO);
    this->declare_parameter<float>("rotation_braking_arc",SETTING_ZERO);
    this->declare_parameter<float>("start_acc",SETTING_ZERO);
    this->declare_parameter<float>("turn_acc",SETTING_ZERO);
    this->declare_parameter<float>("odom_cross_road_init",SETTING_ZERO);
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
    float near_destination_dist;
    float rotation_max_speed;
    float rotation_braking_arc;
    float start_acc;
    float turn_acc;
    float odom_cross_road_init;
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
    this->get_parameter("near_destination_dist",near_destination_dist);
    this->get_parameter("rotation_max_speed",rotation_max_speed);
    this->get_parameter("rotation_braking_arc",rotation_braking_arc);
    this->get_parameter("start_acc",start_acc);
    this->get_parameter("turn_acc",turn_acc);
    this->get_parameter("odom_cross_road_init",odom_cross_road_init);
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
                                                    friction_coefficient,
                                                    near_destination_dist,
                                                    rotation_max_speed,
                                                    rotation_braking_arc,
                                                    start_acc,
                                                    turn_acc,
                                                    odom_cross_road_init);
}


void Center::calculate_straight_movement(float acceleration) {
    if (car_->get_direction() == kec_car::Direction::kBackward) {
        acceleration = -acceleration;
    }
    RCLCPP_INFO(this->get_logger(), "[Center]-[calculate_straight_movement]");
    straight_move_correction(acceleration);
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
#if DEBUG_MODE == 2
    RCLCPP_INFO(this->get_logger(),"[Center]-[route_deviation_callback] offcource_status : %d lat %f log %f, %d distance : %f"
    , status->offcource_status ,status->offcource_goal_lat,status->offcource_goal_lon, car_->get_drive_mode(), status->offcource_goal_distance);
#endif
    // 복구모드 시 갱신 필요 없음.
    // 복구 중 또 들어오는 경우를 방지하기 위함.
    
    /*
    if (car_->get_drive_mode() == kec_car::DrivingMode::kRecovery) {
        return;
    }*/

    if (car_->get_drive_mode() == kec_car::DrivingMode::kRecovery && status->offcource_status==true){
        return;
    }
    else if(car_->get_drive_mode()==kec_car::DrivingMode::kCrossroads){
        devation_status_= std::make_shared<routedevation_msgs::msg::Status>();
        devation_status_->offcource_status=0;
    }
    devation_status_ = status;

}

/**
 * @brief obstacle_status : 0 - drive, 1 - intersection, 2 - parking
 * @brief obstacle_value :
 * @param status
 */
void Center::obstacle_status_callback(const obstacle_msgs::msg::Status::SharedPtr status) {
#if DEBUG_MODE == 2
    std::cout << "[Center]-[obstacle_status_callback] : " <<
              "obstacle_value : " << status->obstacle_value << '\n' <<
              "obstacle_status : " << static_cast<int>(status->obstacle_status) << std::endl;
#endif

    std::unique_ptr<Distance> center_distance = std::make_unique<Distance>();
    route_msgs::msg::DriveBreak temp_break;
    obs_status_ = status;
    if (obs_status_->obstacle_status == 0) {
        if (obs_status_->obstacle_value == 1) {
                // 정지가 필요한 상황
                cmd_stop();
                prev_speed_ =  0;
                if(prev_status_->obstacle_value==status->obstacle_value&&
                prev_status_->obstacle_status==status->obstacle_status){
                    if(obstacle_first_check_==false){
                        // publish
                        obstacle_first_check_=true;
                        pub_obs_event_->publish(*status);
                    }
                }
                else{
                    obstacle_first_check_=false;
                }

        }
    }
    else if (obs_status_->obstacle_status == 2) {
        if (obs_status_->obstacle_value == 1) {
            // 정지가 필요한 상황
            cmd_stop();
            prev_speed_ =  0;
            if(prev_status_->obstacle_value==status->obstacle_value&&
               prev_status_->obstacle_status==status->obstacle_status){
                if(obstacle_first_check_==false){
                    // publish
                    obstacle_first_check_=true;
                    pub_obs_event_->publish(*status);
                }
            }
            else{
                obstacle_first_check_=false;
            }

        } else {
            temp_break.break_pressure = 0;
            pub_break_->publish(temp_break);
        }
    }
    prev_status_= std::shared_ptr<obstacle_msgs::msg::Status>(status);
}

/**
 * @brief robot drive/info timer
 */
void Center::drive_info_timer() {
    DataTypeTrans data_type_trans;
    route_msgs::msg::DriveState drive_state;
    std::string drive_move = data_type_trans.drive_mode_to_string(car_->get_drive_mode());
    drive_state.code = drive_move;
    drive_state.speaker=speaker_seq_++;
    if (car_->get_drive_mode() == kec_car::DrivingMode::kArrive) {
        std::unique_lock<std::mutex> ulm(goal_mutex_);
        cv_.notify_all();
    }
    if (task_ != nullptr) {
        drive_state.start_node = task_->bypass_cur_node_;
        drive_state.end_node = task_->bypass_next_node_;
    }
    RCLCPP_INFO(this->get_logger(),"[Center]-drive_info_timer-car : %s",drive_move.c_str());
    pub_drive_state_->publish(drive_state);
}

void Center::start_on(const std::shared_ptr<RouteToPose::Feedback> feedback,
                      const std::shared_ptr<RouteToPoseGoalHandler> goal_handle) {
    feedback->status_code = static_cast<int>(kec_driving_code::FeedBack::kStart);
    if (!feedback_check_) {
        goal_handle->publish_feedback(feedback);
        feedback_check_ = true;
    }
}

bool Center::cancel_check(const std::shared_ptr<RouteToPose::Result> result,
                          const std::shared_ptr<RouteToPoseGoalHandler> goal_handle) {
    if (goal_handle->is_canceling()) {
        result->result = static_cast<int>(kec_driving_code::Result::kFailedCancel);
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        cancel_check_ = true;
        return true;
    }
    return false;
}

void Center::car_rotation(CarBehavior car_behavior, double node_heading, kec_car::NodeKind node_kind,double init) {
    car_->set_drive_mode(kec_car::DrivingMode::kCrossroads);
    double point_node_heading = node_heading;
    double car_heading = car_->get_degree();
    //float car_speed = car_->get_speed();
    float car_speed = 0.8;
    // 두개가 몇도 차인가?
    double degree = car_behavior.calculate_angle_difference(car_heading,node_heading);
    // 기준점과 차이가 몇도 차인가?
    car_speed = speed_setting(degree,init,ros_parameter_->rotation_braking_arc_);
    prev_speed_ = car_speed;
    #if DEBUG_MODE == 1
                    RCLCPP_INFO(this->get_logger(), "[Center]-[acc speed]-[car_rotation] acc %f, prev %f",car_speed,prev_speed_);
    log_publish(0,0,car_heading,point_node_heading,"Turn","CarRotation");
    #endif
    if (car_behavior.car_move_direct(car_heading,
                                     point_node_heading) == -1) {
        //right
        geometry_msgs::msg::Twist cmd_vel = car_behavior.calculate_rotation_movement(
                car_speed, -ros_parameter_->driving_calibration_max_angle_);
        pub_cmd_->publish(cmd_vel);
        if (node_kind == kec_car::NodeKind::kIntersection) {
            can_msgs::msg::AdControlBody temp_body;
            temp_body.right_turn_light = true;
            temp_body.brake_light = false;
            pub_body_->publish(temp_body);
        }
    } else if (car_behavior.car_move_direct(car_heading,
                                            point_node_heading) == 1) {
        //left
        geometry_msgs::msg::Twist cmd_vel = car_behavior.calculate_rotation_movement(
                car_speed, ros_parameter_->driving_calibration_max_angle_);
        pub_cmd_->publish(cmd_vel);
        if (node_kind == kec_car::NodeKind::kIntersection) {
            can_msgs::msg::AdControlBody temp_body;
            temp_body.left_turn_light = true;
            temp_body.brake_light = false;
            pub_body_->publish(temp_body);
        }
    } else {
#if DEBUG_MODE == 1
        RCLCPP_INFO(this->get_logger(), "[Center]-[car_rotation]-[Success] car_ %f node %f", car_->get_degree(), node_heading);
#endif
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
    double braking_distance = center_distance->distance_braking_calculate(ros_parameter_->max_speed_*3.6,ros_parameter_->friction_coefficient_);

    double straight_brake_pressure=0;
    // 직진 주행을 위한 반복문
    while (rclcpp::ok()) {
        if (cancel_check(result, goal_handle)) {
            return;
        }
        // 도착지 거리
        // 6-1-1) 목적지 도착 여부
        // 자동차 현재 위치 Update
        double car_latitude = car_->get_location().fn_get_latitude();
        double car_longitude = car_->get_location().fn_get_longitude();
        GpsData cur_location;
        cur_location.fn_set_latitude(car_latitude);
        cur_location.fn_set_longitude(car_longitude);
        double goal_distance = center_distance->distance_from_perpendicular_line(task_->get_cur_gps(),
                                                                                 task_->get_next_gps(),
                                                                                 cur_location);

#if DEBUG_MODE == 1
        RCLCPP_INFO(this->get_logger(), "[Center]-[Dist]-[straight_move] init %lf - goal %lf", init_distance, goal_distance);
        log_publish(init_distance,goal_distance,car_->get_degree(),task_->get_cur_heading(),"Straight","ING");
#endif
        // 6-1-1) 목적지 도착 여부
        if (goal_distance <= ros_parameter_->goal_distance_ || goal_distance > init_distance+1) {
#if DEBUG_MODE == 1
            RCLCPP_INFO(this->get_logger(), "[goal]");
            log_publish(init_distance,goal_distance,car_->get_degree(),task_->get_cur_heading(),"Straight","SUCCESS");
#endif

            //cmd_stop();
            car_behavior.cmd_slowly_stop(pub_cmd_,pub_break_);
            prev_speed_ = 0;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            break;
        }

        // 노드 인근 감속
        //car_behavior.determine_brake_pressure(init_distance,goal_distance,car_->get_speed(),ros_parameter_->max_speed_,&straight_brake_pressure,pub_break_);


        // 6-1-2) 이탈 정보 수신 -- route_deviation_callback
        // 6-1-3) 이탈 되었는가?
        routedevation_msgs::msg::Status temp_devation_status;
        {
            mutex_.lock();
            temp_devation_status = *devation_status_;
            mutex_.unlock();
        }
        if(task_->get_cur_dir()==kec_car::Direction::kBackward){
            temp_devation_status.offcource_status=0;
        }
        if(goal_distance<ros_parameter_->near_destination_dist_){
            temp_devation_status.offcource_status=false;
        }
        if (temp_devation_status.offcource_status) {
            kec_car::Mission recovery_result = recovery_move(temp_devation_status,feedback,result,goal_handle,kec_car::DrivingMode::kStraight);
            if(recovery_result == kec_car::Mission::kFAILED){
                return;
            }
        }// 6-1-3-N) 이탈 되었는가?
        else {
            //직진 주행 추가
            //calculate_straight_movement(0.2);
            car_->set_drive_mode(kec_car::DrivingMode::kStraight);
            float speed = speed_setting(static_cast<float>(goal_distance),init_distance,static_cast<float>(braking_distance));
            prev_speed_= speed;

    #if DEBUG_MODE == 1
                    RCLCPP_INFO(this->get_logger(), "[Center]-[acc speed]-[straight] acc %f, prev %f",speed,prev_speed_);
    #endif
            calculate_straight_movement(speed);
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
    std::unique_ptr<Distance> center_distance = std::make_unique<Distance>();
    double turn_straight_init_distance =0;
    double odom_goal_dist = 0;
    if(task_->get_cur_driving_option() == kec_car::DrivingOption::kGps) {
        turn_straight_init_distance = center_distance->distance_from_perpendicular_line(
                task_->get_cur_gps(), task_->get_next_gps(), car_->get_location());
#if DEBUG_MODE == 1
        RCLCPP_INFO(this->get_logger(), "[Center]-[Dist]-[turn_move_straight]-[gps] init %lf", turn_straight_init_distance);
        log_publish(turn_straight_init_distance,0,car_->get_degree(),task_->get_cur_heading(),"turn_straight","gps_setting");
#endif
    }
    else if(task_->get_cur_driving_option() == kec_car::DrivingOption::kOdom){
        turn_straight_init_distance =  center_distance->distance_from_perpendicular_line(task_->get_cur_gps(),task_->get_next_gps(),task_->get_cur_gps());
        odom_goal_dist = car_->get_odom_location();
        if(task_->get_cur_node_kind()==kec_car::NodeKind::kIntersection){
    	    turn_straight_init_distance -=6.3;
        }
#if DEBUG_MODE == 1
        RCLCPP_INFO(this->get_logger(), "[Center]-[Dist]-[turn_move_straight]-[odom] init %lf - goal %lf, odom %lf", turn_straight_init_distance, odom_goal_dist);
        log_publish(turn_straight_init_distance,odom_goal_dist,car_->get_degree(),task_->get_cur_heading(),"turn_straight","odom_setting");
#endif
    }
    else{
        return;
    }
    car_->set_drive_mode(kec_car::DrivingMode::kCrossroads);
    double braking_distance= center_distance->distance_braking_calculate(ros_parameter_->max_speed_*3.6,ros_parameter_->friction_coefficient_);
    while (rclcpp::ok()) {
            if (cancel_check(result, goal_handle)) {
                return;
            }
            if (task_->get_cur_node_kind() == kec_car::NodeKind::kIntersection) {
                car_->set_drive_mode(kec_car::DrivingMode::kCrossroads);
            }
            double goal_distance=0;
            if(task_->get_cur_driving_option() == kec_car::DrivingOption::kGps) {
                goal_distance = center_distance->distance_from_perpendicular_line(
                        task_->get_cur_gps(), task_->get_next_gps(), car_->get_location());
            }
            else if(task_->get_cur_driving_option() == kec_car::DrivingOption::kOdom){
                //goal_distance = turn_straight_init_distance - car_->get_odom_location();
                goal_distance = turn_straight_init_distance - std::fabs(car_->get_odom_location()-odom_goal_dist);
            }
            // 회전 주행 알림.
            //feedback publish
            start_on(feedback, goal_handle);
#if DEBUG_MODE == 1
        RCLCPP_INFO(this->get_logger(), "[Center]-[Dist]-[turn_move_straight]-[ING] init %lf - goal %lf", turn_straight_init_distance, goal_distance);
        log_publish(turn_straight_init_distance,goal_distance,car_->get_degree(),task_->get_cur_heading(),"turn_straight","ING");
#endif

            // 2-6) 직진 목적지에 도착했는가?

            // 회전 중 틀어지지 않도록 task_->rotation.. 을 통해 목적지 판단 무시
            // 최초에는 목적지 도착하여 if문, 이후에는 check를 통해 진입
            if (goal_distance < (ros_parameter_->rotation_straight_dist_) ||
                (task_->rotation_straight_check_)) {
                // 2-6-1) 도착함
                // 2-7) 교차로 노드 헤딩 정보로 방향 확인
#if DEBUG_MODE == 1
                RCLCPP_INFO(this->get_logger(), "[turn_move mode] Turn goal Success");
#endif
                task_->rotation_straight_check_ = true;
                double next_node_heading = task_->get_next_heading();
                double car_heading = car_->get_degree();
                // 2-8) 회전 여부 확인
                if (car_behavior.car_rotation_judgment(
                        car_heading,
                        next_node_heading,
                        ros_parameter_->rotation_angle_tolerance_) == false) {
                    car_rotation(car_behavior, next_node_heading, task_->get_next_node_kind(),-1);

                    //
                } else {
                    // 2-9) 종료
                    car_->set_drive_mode(kec_car::DrivingMode::kStop);
/*                    geometry_msgs::msg::Twist turn_success;
                    turn_success.linear.x = 0;
                    turn_success.angular.z = 0;
                    pub_cmd_->publish(turn_success);*/
                    car_behavior.cmd_slowly_stop(pub_cmd_,pub_break_);
                    task_->rotation_straight_check_ = false;
                    #if DEBUG_MODE ==1 
                    log_publish(turn_straight_init_distance,goal_distance,car_->get_degree(),task_->get_cur_heading(),"turn_straight","SUCCESS");
                    #endif 
                    break;
                }

            } else {
                //2-6-2) 도착하지 않음
#if DEBUG_MODE == 1
                RCLCPP_INFO(this->get_logger(), "[turn_move mode]-[straight] start");
#endif
                car_->set_drive_mode(kec_car::DrivingMode::kStraight);
                routedevation_msgs::msg::Status temp_devation_status;
                {       
                            mutex_.lock();
                            temp_devation_status = *devation_status_;
                            mutex_.unlock();
                }
                        if(task_->get_cur_dir()==kec_car::Direction::kBackward|| task_->get_cur_driving_option()==kec_car::DrivingOption::kOdom){
                            temp_devation_status.offcource_status=0;
                        }
                        if(goal_distance<ros_parameter_->near_destination_dist_){
                            temp_devation_status.offcource_status=false;
                        }
                if (temp_devation_status.offcource_status) {
                            kec_car::Mission recovery_result = recovery_move(temp_devation_status,feedback,result,goal_handle,kec_car::DrivingMode::kCrossroads);
                            if(recovery_result==kec_car::Mission::kFAILED){
                                return;
                            }
                            task_->rotation_straight_check_= true;
                }// 6-1-3-N) 이탈 되었는가?

                else{
                    float speed = speed_setting(static_cast<float>(goal_distance), turn_straight_init_distance, static_cast<float>(braking_distance)+1);
                    prev_speed_ = speed;
    #if DEBUG_MODE == 1
                    RCLCPP_INFO(this->get_logger(), "[Center]-[acc speed]-[turn_move] acc %f, prev %f",speed,prev_speed_);
    #endif
                    calculate_straight_movement(speed);
                
                }

            } // 2-6-2)

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void Center::velocity_status_callback(const robot_status_msgs::msg::VelocityStatus::SharedPtr status) {
    car_->set_speed(status->current_velocity);
}


void Center::odom_eular_callback(const geometry_msgs::msg::PoseStamped::SharedPtr odom_eular) {
    car_->set_odom_location(odom_eular->pose.position.x);
    car_->set_degree(odom_eular->pose.orientation.y);
}

void Center::odom_move(const std::shared_ptr<RouteToPose::Feedback> feedback,
                       const std::shared_ptr<RouteToPose::Result> result,
                       const std::shared_ptr<RouteToPoseGoalHandler> goal_handle) {
    std::unique_ptr<Distance> center_distance = std::make_unique<Distance>();

    // goal distance setting
    double init_goal_dist =  center_distance->distance_from_perpendicular_line(task_->get_cur_gps(), task_->get_next_gps(), task_->get_cur_gps());
    if(task_->get_cur_node_kind()==kec_car::NodeKind::kIntersection){
        init_goal_dist-=6.3;
    }
#if DEBUG_MODE == 1
    RCLCPP_INFO(this->get_logger(), "[Center]-[Dist]-[odom_move] init %lf, setting_odom %lf", init_goal_dist,car_->get_odom_location());
    log_publish(init_goal_dist,car_->get_odom_location(),car_->get_degree(),task_->get_cur_heading(),"odom_move","setting");
#endif

    //double goal_distance =init_goal_dist+car_->get_odom_location();
    double odom_setting_pose =car_->get_odom_location();
    double goal_distance=0;

    double odom_brake_pressure = 0.0;
    Distance distance;
    double braking_distance= distance.distance_braking_calculate(ros_parameter_->max_speed_*3.6,ros_parameter_->friction_coefficient_);
    while (rclcpp::ok()) {
        if (cancel_check(result, goal_handle)) {
            return;
        }
        goal_distance = init_goal_dist - std::fabs(car_->get_odom_location() - odom_setting_pose);
#if DEBUG_MODE == 1
        RCLCPP_INFO(this->get_logger(), "[Center]-[Dist]-[odom_move] init %lf - goal %lf, odom %lf, odom init %lf", init_goal_dist, goal_distance, car_->get_odom_location(),odom_setting_pose);
        log_publish(init_goal_dist,goal_distance,car_->get_degree(),task_->get_cur_heading(),"odom_move","ING");
#endif

        if(goal_distance<ros_parameter_->goal_distance_){
            CarBehavior car_behavior;
            car_behavior.cmd_slowly_stop(pub_cmd_, pub_break_);
            break;
        }
        CarBehavior car_behavior ;
        //car_behavior.determine_brake_pressure(init_goal_dist,goal_distance-car_->get_odom_location(),car_->get_speed(),ros_parameter_->max_speed_,&odom_brake_pressure,pub_break_);

        double acceleration= speed_setting(goal_distance, init_goal_dist, braking_distance);
    #if DEBUG_MODE == 1
                    RCLCPP_INFO(this->get_logger(), "[Center]-[acc speed]-[odom_move] acc %f, prev %f",acceleration,prev_speed_);
    #endif
        prev_speed_ = acceleration;
        if (car_->get_direction() == kec_car::Direction::kBackward) {
            acceleration = -acceleration;
        }
        else{
            car_->set_drive_mode(kec_car::DrivingMode::kStraight);
        }
        #if DEBUG_MODE == 2
        RCLCPP_INFO(this->get_logger(), "[Center]-[odom_move] goal : %f odom :%f init_goal_dist %f acc %f ",goal_distance,car_->get_odom_location(),init_goal_dist,acceleration);
        #endif
        straight_move_correction(static_cast<float>(acceleration));
        start_on(feedback, goal_handle);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void Center::straight_move_correction(float acceleration) {
    geometry_msgs::msg::Twist result_twist;
    result_twist.linear.x = (acceleration);
    result_twist.linear.y = (SETTING_ZERO);
    result_twist.linear.z = (SETTING_ZERO);
    double car_degree = car_->get_degree();
    double node_degree = task_->get_cur_heading();
    CarBehavior car_behavior;
    const double link_degree = car_behavior.calculate_angle_difference(car_degree, node_degree);
    double direction = car_behavior.determine_direction(car_degree,node_degree);
    if(direction>0){
        direction = -ros_parameter_->driving_calibration_angle_increase_;
    }
    else if(direction<0){
        direction = ros_parameter_->driving_calibration_angle_increase_;
    }
    else if(link_degree>ros_parameter_->driving_calibration_min_angle_){
        direction=0;
    }
    if (car_->get_direction() == kec_car::Direction::kBackward) {
        direction=direction*-1;
    }
    const double first_zone = ros_parameter_->driving_calibration_max_angle_ * FIRST_ZONE;
    const double second_zone = ros_parameter_->driving_calibration_max_angle_ * SECOND_ZONE;
    RCLCPP_INFO(this->get_logger(), "[Center]-[straight_move_correction] link _ degree :  %f car %f node %f dir %f", link_degree, car_degree, node_degree,direction);
    if (link_degree > ros_parameter_->driving_calibration_min_angle_) {
        if (link_degree < first_zone) {
            result_twist.angular.z = direction;
        } else if (link_degree > first_zone &&
                   link_degree < second_zone) {
            result_twist.angular.z = direction * 2;
        } else if (link_degree > second_zone) {
            result_twist.angular.z = direction * 3;
        }
    }
    pub_cmd_->publish(result_twist);
}

void Center::cmd_stop() {
    geometry_msgs::msg::Twist stop_twist;
    stop_twist.linear.x=0;
    stop_twist.linear.y=0;
    stop_twist.angular.z=0;
    pub_cmd_->publish(stop_twist);
    route_msgs::msg::DriveBreak temp_break;
    temp_break.break_pressure=100;
    pub_break_->publish(temp_break);
    prev_speed_ = 0;
}

void Center::brake_unlock() {
    route_msgs::msg::DriveBreak temp_break;
    temp_break.break_pressure=0;
    pub_break_->publish(temp_break);
}

float Center::speed_setting(const float goal_dist, const float init_dist, const float brake_dist) {
    float cur_speed = 0;
    float max_speed = 0;

#if DEBUG_MODE == 1
    RCLCPP_INFO(this->get_logger(), "[Center]-[speed_setting] goal %lf - brake %lf , prev %f, init %f", goal_dist,brake_dist,prev_speed_,init_dist );
#endif
    if(init_dist==-1){
        max_speed = ros_parameter_->rotation_max_speed_;
    }
    else{
        max_speed = ros_parameter_->max_speed_;
    }
    //== settting

    if(init_dist <= brake_dist && init_dist!=-1){
        #if DEBUG_MODE == 1
            RCLCPP_INFO(this->get_logger(), "[Center]-[speed_setting] low init dist %f, brake_dist",init_dist,brake_dist);
        #endif
        return 0.4;
    }
    if(goal_dist > brake_dist) {
        if (prev_speed_ >= max_speed) {
            cur_speed = max_speed;
        } else {
            if(init_dist==-1){
                #if DEBUG_MODE == 1
            RCLCPP_INFO(this->get_logger(), "[Center]-[speed_setting] prev %f, turn acc",prev_speed_,ros_parameter_->turn_acc_);
                #endif
                cur_speed = prev_speed_+ros_parameter_->turn_acc_;
            }
            else{
                cur_speed = prev_speed_+ros_parameter_->start_acc_;
            }
        }
    }
    else{
        if(std::fabs(car_->get_speed()) < max_speed/3.6){
            max_speed = std::fabs(car_->get_speed()/3.6);
        }
        cur_speed = max_speed * goal_dist/brake_dist;
        if(cur_speed <= 0.20001){
            cur_speed = 0.2;
        }
#if DEBUG_MODE == 1
        RCLCPP_INFO(this->get_logger(), "[Center]-[speed_setting]-[Result] cur_speed %f cur_car %f", cur_speed,car_->get_speed() );
#endif
    }
    return cur_speed;
}


kec_car::Mission Center::recovery_move(routedevation_msgs::msg::Status devation_status,const std::shared_ptr<RouteToPose::Feedback> feedback,
                           const std::shared_ptr<RouteToPose::Result> result,
                           const std::shared_ptr<RouteToPoseGoalHandler> goal_handle,
                           kec_car::DrivingMode mode){
    #if DEBUG_MODE == 1
        RCLCPP_INFO(this->get_logger(), "[Center]-[recovery_move]-[Start]");
    #endif
    std::unique_ptr<Distance> center_distance = std::make_unique<Distance>();
    double braking_distance = center_distance->distance_braking_calculate(ros_parameter_->max_speed_*3.6,ros_parameter_->friction_coefficient_);

    car_->set_drive_mode(kec_car::DrivingMode::kRecovery);
    CarBehavior car_behavior;
       if (car_behavior.car_rotation_judgment(car_->get_degree(), task_->get_cur_heading(), 45) == false) {
                if (rclcpp::ok()) {
                    //result->result = static_cast<int>(kec_driving_code::Result::kFailedErrorRoute);
                    //goal_handle->abort(result);
                    RCLCPP_INFO(this->get_logger(), "Goal Failed");
                    //failed_check_ = true;
                    return kec_car::Mission::kSUCCESS;
                }
        }
        else {
                init_distance -=ros_parameter_->rotation_straight_dist_;        
                while (rclcpp::ok()) {
                    if (cancel_check(result, goal_handle)) {
                        return kec_car::Mission::kFAILED;
                    }
                    // Goal Gps Setting 
                    GpsData gps_data(devation_status_->offcource_goal_lat,
                    devation_status_->offcource_goal_lon);

                    GpsData temp_car_degree = car_->get_location();
                    GpsData temp_goal_degree = gps_data;

                      // Goal Angle Setting
                    //double goal_angle = center_distance->calculate_line_angle(temp_car_degree, temp_goal_degree);
                    //task_->set_cur_degree(static_cast<double>(goal_angle));
                    double goal_angle=task_->get_cur_heading();
                    if(devation_status_->offcource_status==true){
                        goal_angle = center_distance->calculate_line_angle(temp_car_degree, temp_goal_degree);
                        task_->set_cur_degree(static_cast<double>(goal_angle));
                    }

		    // 6-1-5) 복귀 목적지 도착 여부 N
                    double recovery_goal_distance = center_distance->distance_from_perpendicular_line(
                            task_->get_cur_gps(), gps_data,
                            car_->get_location());

                    if (car_behavior.car_rotation_judgment(car_->get_degree(), task_->get_cur_heading(), 45) == false) {
                        if (rclcpp::ok()) {
                            result->result = static_cast<int>(kec_driving_code::Result::kFailedErrorRoute);
                            goal_handle->abort(result);
                            RCLCPP_INFO(this->get_logger(), "Goal Failed");
                            failed_check_ = true;
                            return kec_car::Mission::kFAILED;
                        }
                    }

#if DEBUG_MODE == 1
                    RCLCPP_INFO(this->get_logger(), "[Center]-[straight_move]-[recovery mode]-[setting] angle : %f, gps-lat %f gps-long %f goal-dist %f\n"
                    "car-lat %f car-long %f",
                    goal_angle,
                    gps_data.fn_get_latitude(),
                    gps_data.fn_get_longitude(),
                    recovery_goal_distance,
                    car_->get_location().fn_get_latitude(),
                    car_->get_location().fn_get_longitude());
                    log_publish(0,recovery_goal_distance,car_->get_degree(),goal_angle,"recovery_move","ING");

#endif
                    // 복구 도착지 판단
                    double recovery_distance_tolerance =0;
                    if(kec_car::DrivingMode::kStraight == mode){
                        recovery_distance_tolerance= ros_parameter_->recovery_goal_tolerance_;
                    }
                    else if(kec_car::DrivingMode::kCrossroads == mode){
                        recovery_distance_tolerance = ros_parameter_->rotation_straight_dist_;
                    }


                    if (recovery_distance_tolerance < recovery_goal_distance) {
                            float speed = speed_setting(static_cast<float>(recovery_goal_distance) ,init_distance,static_cast<float>(braking_distance));
                            prev_speed_ = speed;
    #if DEBUG_MODE == 1
                    RCLCPP_INFO(this->get_logger(), "[Center]-[acc speed]-[recovery] acc %f, prev %f",speed,prev_speed_);
    #endif
                            calculate_straight_movement(
                                    speed);
                        
                    }// 6-1-5) 복귀 목적지 도착 여부 N
                    else {
#if DEBUG_MODE == 1
                        RCLCPP_INFO(this->get_logger(), "[Center]-[recovery mode] goal successed");
                        log_publish(0,recovery_goal_distance,car_->get_degree(),goal_angle,"recovery_move","Success");
#endif
                        car_->set_drive_mode(kec_car::DrivingMode::kStop);
                        car_behavior.cmd_slowly_stop(pub_cmd_,pub_break_);
                        break;
                    }// 6-1-5) 복귀 목적지 도착 여부 Y
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }

        }
    return kec_car::Mission::kSUCCESS;
}

void Center::log_publish(double init_dist, double cur_dist,double car_degree ,double node_degree,std::string mode, std::string note){
    route_msgs::msg::Log temp_log;
    temp_log.stamp = this->now();
    temp_log.init_dist = init_dist;
    temp_log.cur_dist = cur_dist;
    temp_log.car_degree = car_degree;
    temp_log.node_degree = node_degree;
    temp_log.mode = std::move(mode);
    temp_log.note = std::move(note);
    pub_log_->publish(temp_log);
}

std::string Center::get_time() {
    auto now = this->now();
    std::stringstream ss;
    ss << std::fixed << std::setprecision(9) << now.seconds();
    return ss.str();
}
