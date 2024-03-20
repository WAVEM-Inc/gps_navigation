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
        /*kec_car::NodeKind start_kind = car_mode_determine(start_node_kind);
     kec_car::NodeKind end_kind = car_mode_determine(end_node_kind);*/
        // 연결-연결, 연결-완료, 연결-주행 종료, 연결-일시 정지, 교차로-연결, 교차로-완료, 교차로-주행 종료, 교차로-일시 정지,
        // 완료-연결, 완료-완료, 완료-주행 종료, 완료-일시 정지, 종점-연결 조건을 확인하여 직진 여부 판단
        if ((start_kind == kec_car::NodeKind::kConnecting && end_kind == kec_car::NodeKind::kConnecting) ||
            (start_kind == kec_car::NodeKind::kConnecting && end_kind == kec_car::NodeKind::kComplete) ||
            (start_kind == kec_car::NodeKind::kConnecting && end_kind == kec_car::NodeKind::kEndpoint) ||
            (start_kind == kec_car::NodeKind::kConnecting && end_kind == kec_car::NodeKind::kWaiting) ||
            (start_kind == kec_car::NodeKind::kIntersection && end_kind == kec_car::NodeKind::kConnecting) ||
            (start_kind == kec_car::NodeKind::kIntersection && end_kind == kec_car::NodeKind::kComplete) ||
            (start_kind == kec_car::NodeKind::kIntersection && end_kind == kec_car::NodeKind::kEndpoint) ||
            (start_kind == kec_car::NodeKind::kIntersection && end_kind == kec_car::NodeKind::kWaiting) ||
            (start_kind == kec_car::NodeKind::kComplete && end_kind == kec_car::NodeKind::kConnecting) ||
            (start_kind == kec_car::NodeKind::kComplete && end_kind == kec_car::NodeKind::kComplete) ||
            (start_kind == kec_car::NodeKind::kComplete && end_kind == kec_car::NodeKind::kEndpoint) ||
            (start_kind == kec_car::NodeKind::kComplete && end_kind == kec_car::NodeKind::kWaiting) ||
            (start_kind == kec_car::NodeKind::kEndpoint && end_kind == kec_car::NodeKind::kConnecting)) {
                return true;
        } else {
                return false;
        }
}

bool CarBehavior::intersection_judgment(kec_car::NodeKind start_kind, kec_car::NodeKind end_kind) {
        if ((start_kind==kec_car::NodeKind::kIntersection && end_kind==kec_car::NodeKind::kIntersection)||
            (start_kind==kec_car::NodeKind::kWaiting && end_kind==kec_car::NodeKind::kIntersection)){
                return true;
        } else {
                return false;
        }
}
