//
// Created by nuc-bt on 24. 3. 6.
//

#ifndef ROUTE_TRACKER_ROS_PARAMETER_HPP
#define ROUTE_TRACKER_ROS_PARAMETER_HPP


class RosParameter {
public:
    explicit RosParameter(float imu_correction) : imu_correction_(imu_correction) {
    }

public :
    float imu_correction_;
};


#endif //ROUTE_TRACKER_ROS_PARAMETER_HPP
