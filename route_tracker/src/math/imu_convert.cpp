//
// Created by nuc-bt on 24. 3. 6.
//

#include "imu_convert.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
float ImuConvert::get_correction() const {
    return correction_;
}

void ImuConvert::set_correction(float correction) {
    correction_ = correction;
}

double ImuConvert::quaternion_to_heading_converter(const sensor_msgs::msg::Imu::SharedPtr imu) {
    tf2::Quaternion tf_quaternion(
            imu->orientation.x, imu->orientation.y, imu->orientation.z, imu->orientation.w);
    tf2::Matrix3x3 matrix(tf_quaternion);
    double roll=0,pitch=0,yaw=0;
    matrix.getRPY(roll,pitch,yaw);
    return yaw+correction_;
}
