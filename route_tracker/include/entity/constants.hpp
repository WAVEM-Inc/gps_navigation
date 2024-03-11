//
// Created by nuc-bt on 24. 3. 6.
//

#ifndef ROUTE_TRACKER_CONSTANTS_HPP
#define ROUTE_TRACKER_CONSTANTS_HPP

#define TP_NAME_ROUTE_TO_POSE "/drive/route_to_pose"
#define TP_NAME_IMU "/sensor/imu/data"
#define TP_NAME_GPS "/sensor/ublox/fix"
#define TP_NAME_CMD_VEL "/drive/cmd_vel"

#include<iostream>
class Constants {
public:
    explicit Constants() :
        tp_name_route_to_pose_(TP_NAME_ROUTE_TO_POSE),
        tp_name_imu_(TP_NAME_IMU),
        tp_name_gps_(TP_NAME_GPS),
        tp_name_cmd_(TP_NAME_CMD_VEL){}

public :
    const std::string tp_name_route_to_pose_;
    const std::string tp_name_imu_;
    const std::string tp_name_gps_;
    const std::string tp_name_cmd_;
};


#endif //ROUTE_TRACKER_CONSTANTS_HPP
