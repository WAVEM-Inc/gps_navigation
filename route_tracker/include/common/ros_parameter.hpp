//
// Created by nuc-bt on 24. 3. 6.
//

#ifndef ROUTE_TRACKER_ROS_PARAMETER_HPP
#define ROUTE_TRACKER_ROS_PARAMETER_HPP

#include "common/test.h"

class RosParameter {
public:
    explicit RosParameter(float imu_correction,
                          float max_speed,
                          float driving_calibration_max_angle,
                          float driving_calibration_min_angle,
                          float driving_calibration_angle_increase,
                          float goal_distance,
                          float rotation_straight_dist,
                          float rotation_angle_increase,
                          float rotation_angle_tolerance,
                          float recovery_goal_tolerance,
                          float deceleration,
                          float friction_coefficient,
                          float near_destination_dist,
                          float rotation_max_speed,
                          float rotation_braking_arc,
                          float start_acc,
                          float turn_acc,
                          float odom_cross_road_init)
            : imu_correction_(imu_correction),
              max_speed_(max_speed),
              driving_calibration_max_angle_(driving_calibration_max_angle),
              driving_calibration_min_angle_(driving_calibration_min_angle),
              driving_calibration_angle_increase_(driving_calibration_angle_increase),
              goal_distance_(goal_distance),
              rotation_straight_dist_(rotation_straight_dist),
              rotation_angle_increase_(rotation_angle_increase),
              rotation_angle_tolerance_(rotation_angle_tolerance),
              recovery_goal_tolerance_(recovery_goal_tolerance),
              deceleration_(deceleration),
              friction_coefficient_(friction_coefficient),
              near_destination_dist_(near_destination_dist),
              rotation_max_speed_(rotation_max_speed),
              rotation_braking_arc_(rotation_braking_arc),
              start_acc_(start_acc),
              turn_acc_(turn_acc),
              odom_cross_road_init_(odom_cross_road_init){
#if DEBUG_MODE == 1
            std::cout << "imu_correction_ : " << imu_correction << '\n' <<
                      "max_speed_ : " << max_speed << '\n' <<
                      "driving_calibration_max_angle_ : " << driving_calibration_max_angle << '\n' <<
                      "driving_calibration_min_angle_ : " << driving_calibration_min_angle << '\n' <<
                      "driving_calibration_angle_increase_ : " << driving_calibration_angle_increase << '\n' <<
                      "goal_distance_ : " << goal_distance << '\n' <<
                      "rotation_straight_dist_ : " << rotation_straight_dist << '\n' <<
                      "rotation_angle_increase_ : " << rotation_angle_increase << '\n' <<
                      "rotation_angle_tolerance_ : " << rotation_angle_tolerance << '\n' <<
                      "recovery_goal_tolerance_ : " << recovery_goal_tolerance << '\n' <<
                      "deceleration_ : " << deceleration << '\n' <<
                      "friction_coefficient_ : " << friction_coefficient <<'\n'<<
                      "near_destination_dist_ : "<<near_destination_dist<<'\n' <<
                      "rotation_max_speed : "<<rotation_max_speed<<'\n'<<
                      "rotation_braking_arc : "<<rotation_braking_arc<<'\n'<<
                      "start_acc : " << start_acc<<'\n'<<
                      "turn acc : "<<turn_acc<<'\n'<<
                      "odom_cross_road_init : " <<odom_cross_road_init<<std::endl;
#endif
    }

public :
    float imu_correction_;
    float max_speed_;
    float driving_calibration_max_angle_;
    float driving_calibration_min_angle_;
    float driving_calibration_angle_increase_;
    float goal_distance_;
    float rotation_straight_dist_;
    float rotation_angle_increase_;
    float rotation_angle_tolerance_;
    float recovery_goal_tolerance_;
    float deceleration_;
    float friction_coefficient_;
    float near_destination_dist_;
    float rotation_max_speed_;
    float rotation_braking_arc_;
    float start_acc_;
    float turn_acc_;
    float odom_cross_road_init_;
};


#endif //ROUTE_TRACKER_ROS_PARAMETER_HPP
