//
// Created by nuc-bt on 24. 3. 20.
//

#ifndef ROUTE_TRACKER_CAR_BEHAVIOR_HPP
#define ROUTE_TRACKER_CAR_BEHAVIOR_HPP
#include "geometry_msgs/msg/twist.hpp"
#include "code/kec_car_data.hpp"
#include "route_msgs/msg/drive_break.hpp"
#include "rclcpp/rclcpp.hpp"

enum class quadrant{
        kFirst,
        kSecond,
        kThird,
        kFourth
};
class CarBehavior {
private :
    quadrant find_angle_quadrant(double degree);
    double normalize_angle(double angle);
public:
    CarBehavior();
    int car_move_direct(double car_degree, double next_node_degree);
    geometry_msgs::msg::Twist calculate_rotation_movement(float linear,float angle);
    bool car_rotation_judgment(double car_degree ,double node_degree , double angle_tolerance);
    bool straight_judgment(kec_car::NodeKind start_kind, kec_car::NodeKind end_kind);
    bool intersection_judgment(kec_car::NodeKind start_kind, kec_car::NodeKind end_kind);
    bool waiting_judgment(kec_car::NodeKind start_kind);
    double calculate_angle_difference(double current_angle, double exit_angle);
    int determine_direction(double base_angle, double target_angle);
    void determine_brake_pressure(double init_dist, double remaining_dist, double car_speed, const double max_speed,double* prev_brake_pressure, rclcpp::Publisher<route_msgs::msg::DriveBreak>::SharedPtr pub_break);
    void cmd_slowly_stop(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd, rclcpp::Publisher<route_msgs::msg::DriveBreak>::SharedPtr pub_brake);
};


#endif //ROUTE_TRACKER_CAR_BEHAVIOR_HPP
