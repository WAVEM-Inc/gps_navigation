//
// Created by nuc-bt on 24. 3. 6.
//

#ifndef ROUTE_TRACKER_IMU_CONVERT_HPP
#define ROUTE_TRACKER_IMU_CONVERT_HPP

#include"sensor_msgs/msg/imu.hpp"
class ImuConvert {
private :
    float correction_;
public:
    float get_correction() const;
    void set_correction(float correction);
    double quaternion_to_heading_converter(const sensor_msgs::msg::Imu::SharedPtr imu);
};


#endif //ROUTE_TRACKER_IMU_CONVERT_HPP
