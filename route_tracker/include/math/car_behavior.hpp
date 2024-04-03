//
// Created by nuc-bt on 24. 3. 20.
//

#ifndef ROUTE_TRACKER_CAR_BEHAVIOR_HPP
#define ROUTE_TRACKER_CAR_BEHAVIOR_HPP
#include "geometry_msgs/msg/twist.hpp"
#include "code/kec_car_data.hpp"
enum class quadrant{
        kFirst,
        kSecond,
        kThird,
        kFourth
};
class CarBehavior {
private :
    quadrant find_angle_quadrant(double degree);
public:
    CarBehavior();
    bool car_move_direct(double car_degree, double next_node_degree);
    geometry_msgs::msg::Twist calculate_rotation_movement(float linear,float angle);
    bool car_rotation_judgment(double degree , double angle_tolerance);
    bool straight_judgment(kec_car::NodeKind start_kind, kec_car::NodeKind end_kind);
    bool intersection_judgment(kec_car::NodeKind start_kind, kec_car::NodeKind end_kind);
    bool waiting_judgment(kec_car::NodeKind start_kind);
};


#endif //ROUTE_TRACKER_CAR_BEHAVIOR_HPP
