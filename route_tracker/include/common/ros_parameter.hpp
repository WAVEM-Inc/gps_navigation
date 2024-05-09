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
                          float rotation_max_speed
    )
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
              rotation_max_speed_(rotation_max_speed){
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
                      "rotation_max_speed : "<<rotation_max_speed<<std::endl;
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
};


#endif //ROUTE_TRACKER_ROS_PARAMETER_HPP
