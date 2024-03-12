//
// Created by nuc-bt on 24. 3. 6.
//

#ifndef ROUTE_TRACKER_CONSTANTS_HPP
#define ROUTE_TRACKER_CONSTANTS_HPP

#define TP_NAME_ROUTE_TO_POSE "/drive/route_to_pose"
#define TP_NAME_IMU "/sensor/imu/data"
#define TP_NAME_GPS "/sensor/ublox/fix"
#define TP_NAME_CMD_VEL "/cmd_vel"
#define TP_NAME_ODOM "/odom"
#define TP_NAME_ROUTE_DEVIATION "/drive/route_deviation/status"
#define TP_NAME_OBSTACLE_STATUS "/drive/obstacle/status"
#define TP_NAME_DRIVE_BREAK "/drive/break"

#include<iostream>
class Constants {
public:
    explicit Constants() :
        tp_name_route_to_pose_(TP_NAME_ROUTE_TO_POSE),
        tp_name_imu_(TP_NAME_IMU),
        tp_name_gps_(TP_NAME_GPS),
        tp_name_cmd_(TP_NAME_CMD_VEL),
        tp_name_odom_(TP_NAME_ODOM),
        tp_name_route_deviation_(TP_NAME_ROUTE_DEVIATION),
        tp_name_obstacle_status_(TP_NAME_OBSTACLE_STATUS),
        tp_name_drive_break_(TP_NAME_DRIVE_BREAK)
        {}

public :
    const std::string tp_name_route_to_pose_;
    const std::string tp_name_imu_;
    const std::string tp_name_gps_;
    const std::string tp_name_cmd_;
    const std::string tp_name_odom_;
    const std::string tp_name_route_deviation_;
    const std::string tp_name_obstacle_status_;
    const std::string tp_name_drive_break_;
};


#endif //ROUTE_TRACKER_CONSTANTS_HPP
