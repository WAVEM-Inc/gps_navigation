//
// Created by nuc-bt on 24. 3. 6.
//

#ifndef ROUTE_TRACKER_ROS_PARAMETER_HPP
#define ROUTE_TRACKER_ROS_PARAMETER_HPP


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
                          float rotation_angle_tolerance
    )
            : imu_correction_(imu_correction),
              max_speed_(max_speed),
              driving_calibration_max_angle_(driving_calibration_max_angle),
              driving_calibration_min_angle_(driving_calibration_min_angle),
              driving_calibration_angle_increase_(driving_calibration_angle_increase),
              goal_distance_(goal_distance),
              rotation_straight_dist_(rotation_straight_dist),
              rotation_angle_increase_(rotation_angle_increase),
              rotation_angle_tolerance_(rotation_angle_tolerance){
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
};


#endif //ROUTE_TRACKER_ROS_PARAMETER_HPP
