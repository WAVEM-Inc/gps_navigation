//
// Created by nuc-bt on 24. 3. 20.
//

#include "car_behavior.hpp"
#include <cmath>

#include "common/test.h"
#include "rcutils/logging_macros.h"

CarBehavior::CarBehavior() {

}

/**
 *
 * @param car_degree
 * @param next_node_degree
 * @return  1 : left , -1 : right 0 = same
 */
int CarBehavior::car_move_direct(double car_degree, double next_node_degree) {
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
    if(angle_difference < 0){
        return -1;
    }
    else if(angle_difference > 0){
        return 1;
    }
    else{
        return 0;
    }
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

geometry_msgs::msg::Twist CarBehavior::calculate_rotation_movement(float linear, float angle) {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = linear;
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
bool CarBehavior::car_rotation_judgment(double car_degree, double node_degree, double angle_tolerance) {
    double degree = 0;
    degree = calculate_angle_difference(car_degree, node_degree);
    double diff = calculate_angle_difference(degree, angle_tolerance);
#if DEBUG_MODE == 1
    RCUTILS_LOG_INFO_NAMED("CAR_BEHAVIOR",
                           "[car_rotation_judgment] car_degree %f node_degree %f degree %f tolerance %f diff %f result %d",
                           car_degree, node_degree,
                           degree, angle_tolerance, diff,(diff < angle_tolerance) ? true : false);
#endif

    return (diff < angle_tolerance) ? true : false;
}

bool CarBehavior::straight_judgment(kec_car::NodeKind start_kind, kec_car::NodeKind end_kind) {
    // 목적지가 회전이거나 출발지가 대기인 경우 false
    if (end_kind == kec_car::NodeKind::kIntersection || start_kind == kec_car::NodeKind::kWaiting) {
        return false;
    } else {
        return true;
    }
}

bool CarBehavior::intersection_judgment(kec_car::NodeKind start_kind, kec_car::NodeKind end_kind) {
    if (end_kind == kec_car::NodeKind::kIntersection) {
        return true;
    } else {
        return false;
    }
}

bool CarBehavior::waiting_judgment(kec_car::NodeKind start_kind) {
    if (start_kind == kec_car::NodeKind::kWaiting) {
        return true;
    }
    return false;
}

double CarBehavior::normalize_angle(double angle) {
    // angle을 0부터 360도 사이의 값으로 변환
    angle = fmod(angle, 360.0);
    if (angle < 0) {
        angle += 360.0;
    }
    return angle;
}

double CarBehavior::calculate_angle_difference(double current_angle, double exit_angle) {
/*    // 각도를 0부터 360도 사이의 값으로 정규화
    current_angle = normalizeAngle(current_angle);
    exit_angle = normalizeAngle(exit_angle);

    // 각도 차이 계산
    double angleDifference = exit_angle - current_angle;

    // 음수 각도를 처리하여 0부터 360도 사이의 값으로 변환
    angleDifference = normalize_angle(angleDifference);
#if DEBUG_MODE ==1
    RCUTILS_LOG_INFO_NAMED("CAR_BEHAVIOR","[calculate_angle_difference] current_angle %f exit_angle %f angleDifference %f", current_angle , exit_angle,
                           angleDifference);
#endif
    if (angleDifference > 180.0) {
        angleDifference = 360.0 - angleDifference;
    }
    return angleDifference;*/
    current_angle = normalize_angle(current_angle);
    exit_angle = normalize_angle(exit_angle);

    // 각도 차이 계산
    double angleDifference = exit_angle - current_angle;

    // 음수 각도를 처리하여 0부터 360도 사이의 값으로 변환
    angleDifference = normalize_angle(angleDifference);

    // 180도를 넘어서면 반대 방향으로 계산
    if (angleDifference > 180.0) {
        angleDifference = 360.0 - angleDifference;
    }

    return angleDifference;
}

int CarBehavior::determine_direction(double base_angle, double target_angle) {
    // 각도의 범위를 0도에서 360도로 정규화
    base_angle = std::fmod(base_angle, 360.0);
    target_angle = std::fmod(target_angle, 360.0);

    // 각도 차이 계산
    double angle_difference = target_angle - base_angle;

    // 각도 차이를 -180도에서 180도 범위로 정규화
    if (angle_difference > 180.0) {
        angle_difference -= 360.0;
    }
    else if (angle_difference < -180.0) {
        angle_difference += 360.0;
    }

    // 각도 차이가 음수인지 양수인지를 판별하여 왼쪽 또는 오른쪽 방향을 결정
    if (angle_difference < 0) {
        return 1;  // 기준 각도의 왼쪽에 위치
    }
    else if(angle_difference>0) {
        return -1; // 기준 각도의 오른쪽에 위치
    }
    else{
        return 0;
    }
}