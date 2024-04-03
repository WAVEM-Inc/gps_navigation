//
// Created by nuc-bt on 24. 3. 20.
//

#include "car_behavior.hpp"
#include <cmath>
CarBehavior::CarBehavior() {

}

/**
 *
 * @param car_degree
 * @param next_node_degree
 * @return bool false : left , true : right
 */
bool CarBehavior::car_move_direct(double car_degree, double next_node_degree) {
        double angle_difference = next_node_degree - car_degree;
        // 각도차를 -180 ~ 180 범위 내로 정규화
        angle_difference = std::fmod(angle_difference, 360);
        if (angle_difference > 180) {
                angle_difference -= 360;
        }
        else if (angle_difference < -180) {
                angle_difference += 360;
        }
        // If angle_difference is positive, car should turn right (true)
        // If angle_difference is negative, car should turn left (false)
        return angle_difference < 0;
}
/**
 *
 * @param angle
 * @return
 */
quadrant CarBehavior::find_angle_quadrant(double angle) {
        angle = fmod(angle, 360);
        if (angle < 0) {
                angle += 360;
        }
        // 사분면 결정 로직
        if (angle > 0 && angle <= 90) {
                return quadrant::kFirst;
        } else if (angle > 90 && angle <= 180) {
                return quadrant::kSecond;
        } else if (angle > 180 && angle <= 270) {
                return quadrant::kThird;
        } else {
                return quadrant::kFourth; // 270 < angle <= 360
        }
}

geometry_msgs::msg::Twist CarBehavior::calculate_rotation_movement(float linear,float angle) {
        geometry_msgs::msg::Twist twist;
        twist.linear.x=linear;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = angle;
        return twist;
}

/**
 *
 * @param degree
 * @param angle_tolerance
 * @return bool true : 회전 완료, false : 회전 필요
 */
bool CarBehavior::car_rotation_judgment(double degree, double angle_tolerance) {
        return degree < angle_tolerance;
}

bool CarBehavior::straight_judgment(kec_car::NodeKind start_kind, kec_car::NodeKind end_kind) {
        // 목적지가 회전이거나 출발지가 대기인 경우 false
        if(end_kind==kec_car::NodeKind::kIntersection || start_kind == kec_car::NodeKind::kWaiting){
                return false;
        }
        else{
                return true;
        }
}

bool CarBehavior::intersection_judgment(kec_car::NodeKind start_kind, kec_car::NodeKind end_kind) {
        if (end_kind==kec_car::NodeKind::kIntersection){
                return true;
        } else {
                return false;
        }
}

bool CarBehavior::waiting_judgment(kec_car::NodeKind start_kind) {
        if(start_kind == kec_car::NodeKind::kWaiting){
                return true;
        }
        return false;
}
