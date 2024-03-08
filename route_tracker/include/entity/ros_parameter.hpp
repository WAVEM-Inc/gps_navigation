//
// Created by nuc-bt on 24. 3. 6.
//

#ifndef ROUTE_TRACKER_ROS_PARAMETER_HPP
#define ROUTE_TRACKER_ROS_PARAMETER_HPP


class RosParameter {
public:
    explicit RosParameter(float imu_correction, float max_speed, float driving_calibration_angle) : imu_correction_(imu_correction),max_speed_(max_speed),driving_calibration_angle_(driving_calibration_angle) {
    }

public :
    float imu_correction_;
    float max_speed_;
    float driving_calibration_angle_;
};


#endif //ROUTE_TRACKER_ROS_PARAMETER_HPP
