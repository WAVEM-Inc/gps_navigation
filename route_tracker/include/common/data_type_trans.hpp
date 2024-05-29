//
// Created by nuc-bt on 24. 3. 18.
//

#ifndef ROUTE_TRACKER_DATA_TYPE_TRANS_HPP
#define ROUTE_TRACKER_DATA_TYPE_TRANS_HPP

#include<iostream>
#include "code/kec_car_data.hpp"
#include "entity/task.hpp"

class DataTypeTrans {
public:
    DataTypeTrans() {}

    std::string drive_mode_to_string(const kec_car::DrivingMode mode) {
            switch (mode) {
                    case kec_car::DrivingMode::kStraight:
                            return "straight"; //  "직진"
                    case kec_car::DrivingMode::kRecovery:
                            return "recovery"; //  "복귀"
                    case kec_car::DrivingMode::kStop:
                            return "stop"; //  "정지"
                    case kec_car::DrivingMode::kParking:
                            return "parking"; //  "주차"
                    case kec_car::DrivingMode::kCrossroads:
                            return "crossroads"; //  "교차로"
                    case kec_car::DrivingMode::kArrive:
                            return "arrive";//완료
                    default:
                            return "Unknown Driving Mode";
            }//switch
    }//drive_mode_to_string

    kec_car::NodeKind car_mode_determine(std::string car_node) {
            static const std::unordered_map<std::string, kec_car::NodeKind> node_kind_map = {
                    {"intersection", kec_car::NodeKind::kIntersection},
                    {"connecting",   kec_car::NodeKind::kConnecting},
                    {"complete",     kec_car::NodeKind::kComplete},
                    {"endpoint",     kec_car::NodeKind::kEndpoint},
                    {"waiting",      kec_car::NodeKind::kWaiting}
            };
            auto it = node_kind_map.find(car_node);
            if (it != node_kind_map.end()) {
                    return it->second;
            } else {
                    // 기본값으로 대기 노드 설정
                    return kec_car::NodeKind::kWaiting;
            }
    }
    kec_car::Direction car_direction_determine(const std::string& direction_str) {
        using DIR = kec_car::Direction;
        static const std::unordered_map<std::string, DIR> direction_map = {
                {"forward", DIR::kForward},
                {"backward", DIR::kBackward}
        };
        auto it = direction_map.find(direction_str);
        if (it != direction_map.end()) {
            return it->second;
        } else {
            // 기본값으로 전진 방향 설정
            return DIR::kForward;
        }
    }

    /**
     * @brief Change the string data to a variable in the enum class, use GPS for basic data, but can be changed later
     * @param driving_option_str
     * @return
     */
    kec_car::DrivingOption driving_option_determine(const std::string& driving_option_str){
                std::cout << driving_option_str << std::endl;
            using DO = kec_car::DrivingOption;
            static const std::unordered_map<std::string, DO> driving_option_map = {
                    {"odom", DO::kOdom},
                    {"gps", DO::kGps}
            };
            auto it = driving_option_map.find(driving_option_str);
            if (it != driving_option_map.end()) {
                std::cout << "test"<< std::endl;
                return it->second;
            } else {
                std::cout << "gps"<< std::endl;
                return DO::kGps;
            }
    }
    std::string drive_option_to_string(const kec_car::DrivingOption mode) {
            switch (mode) {
                    case kec_car::DrivingOption::kGps:
                            return "kGps"; //  "직진"
                    case kec_car::DrivingOption::kOdom:
                            return "kOdom"; //  "복귀"
                    default:
                            return "Unknown Driving Mode";
            }//switch
    }//drive_mode_to_string
};


#endif //ROUTE_TRACKER_DATA_TYPE_TRANS_HPP
