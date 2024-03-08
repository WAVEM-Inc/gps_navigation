//
// Created by nuc-bt on 24. 3. 5.
//

#include "route_tracker/center.hpp"

Center::Center() : Node("route_tracker_node") {
    constants_= std::make_unique<Constants>();
    imu_converter_= std::make_unique<ImuConvert>();
    car_= std::make_unique<Car>();
    ros_parameter_setting();
    ros_init();
}

void Center::ros_init() {
    auto default_qos= rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    // action_server
    rclcpp::CallbackGroup::SharedPtr callback_group_action_server;
    callback_group_action_server = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    route_to_pose_action_server_ = rclcpp_action::create_server<RouteToPose>(
            this,
            constants_->tp_name_route_to_pose_,
            std::bind(&Center::route_to_pose_goal_handle, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&Center::route_to_pose_cancel_handle, this, std::placeholders::_1),
            std::bind(&Center::route_to_pose_accepted_handle, this, std::placeholders::_1),
            rcl_action_server_get_default_options(),
            callback_group_action_server
    );
    // imu callback
    rclcpp::CallbackGroup::SharedPtr cb_group_imu;
    cb_group_imu = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_imu_options;
    sub_imu_options.callback_group = cb_group_imu;
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(constants_->tp_name_imu_,default_qos,std::bind(&Center::imu_callback,this,std::placeholders::_1),sub_imu_options);

    // gps callback
    rclcpp::CallbackGroup::SharedPtr cb_group_gps;
    cb_group_gps = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_gps_options;
    sub_gps_options.callback_group = cb_group_gps;
    sub_gps_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(constants_->tp_name_gps_,default_qos,std::bind(&Center::gps_callback,this,std::placeholders::_1),sub_gps_options);

    // 장애물 정보 callback
    // 경로 이탈 정보 callback
    // 로봇 모드 timer
    //   ㄴ speaker timer
}

void Center::fn_run() {

}

rclcpp_action::GoalResponse
Center::route_to_pose_goal_handle(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const RouteToPose::Goal> goal) {
    // reset
    if(cur_node_.use_count()>0) {
        cur_node_.reset();
    }
    cur_node_=std::make_shared<route_msgs::msg::Node>(goal->start_node) ;

    // 노트 타입에 따라 입력
    car_->set_node_kind(car_mode_determine(cur_node_->kind));

    // 연결 노드 일때 45도 이상 전환하지 못하도록
    if(car_->get_node_kind()==kec_car::NodeKind::kConnecting) {
        // abs(출발지 노드 진출 방향-기존 노드 진출 방향) >45
            return std::abs(car_->get_degree() - cur_node_->heading) > 45 ? rclcpp_action::GoalResponse::REJECT
                                                                           : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
Center::route_to_pose_cancel_handle(const std::shared_ptr<RouteToPoseGoalHandler> goal_handle) {
    return rclcpp_action::CancelResponse::ACCEPT;
}

void Center::route_to_pose_accepted_handle(const std::shared_ptr<RouteToPoseGoalHandler> goal_handle) {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&Center::route_to_pose_execute, this, std::placeholders::_1), goal_handle}.detach();
}

void Center::route_to_pose_execute(const std::shared_ptr<RouteToPoseGoalHandler> goal_handler) {

    // max speed

}

/**
 * @brief imu callback, 차량 각도 제공
 * @param imu
 */
void Center::imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu) {
    imu_converter_->set_correction(ros_parameter_->imu_correction_);
    car_->set_degree( static_cast<float>(imu_converter_->quaternion_to_heading_converter(imu)));
}

void Center::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr gps) {

}

void Center::ros_parameter_setting() {
    this->declare_parameter<float>("imu_correction",0.0);
    this->declare_parameter<float>("max_speed",0.0);
    this->declare_parameter<float>("driving_calibration_angle",0.0);
    float imu_correction;
    float max_speed;
    float driving_calibration_angle;
    if(this->get_parameter("imu_correction",imu_correction)){
        std::cout<<imu_correction<<std::endl;
    }
    if(this->get_parameter("max_speed",max_speed)){
        std::cout<<max_speed<<std::endl;
    }
    if(this->get_parameter("driving_calibration_angle",driving_calibration_angle)){
        std::cout<<driving_calibration_angle<<std::endl;
    }
    ros_parameter_= std::make_unique<RosParameter>(imu_correction,max_speed,driving_calibration_angle);
}

kec_car::NodeKind Center::car_mode_determine(std::string car_node) {
    static const std::unordered_map<std::string, kec_car::NodeKind> node_kind_map = {
            {"intersection", kec_car::NodeKind::kIntersection},
            {"connecting", kec_car::NodeKind::kConnecting},
            {"endpoint", kec_car::NodeKind::kEndpoint},
            {"waiting", kec_car::NodeKind::kWating}
    };
    auto it = node_kind_map.find(car_node);
    if (it != node_kind_map.end()) {
        return it->second;
    } else {
        // 기본값으로 대기 노드 설정
        return kec_car::NodeKind::kWating;
    }
}
