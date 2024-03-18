//
// Created by nuc-bt on 24. 3. 18.
//

#ifndef ROUTE_TRACKER_DATA_TYPE_TRANS_HPP
#define ROUTE_TRACKER_DATA_TYPE_TRANS_HPP

#include<iostream>
#include "code/kec_car_data.hpp"
#include "entity/task.hpp"

class TaskGoal;

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
                    default:
                            return "Unknown Driving Mode";
            }//switch
    }//drive_mode_to_string

    kec_car::NodeKind string_to_node_kind(const std::string data) const {

    }

    /**
     *
     * @param kind
     * @return
     */
    /**
   bool straight_judgment(kec_car::NodeKind kind){
       //

       //
       return (kind == kec_car::NodeKind::kConnecting ||
               kind == kec_car::NodeKind::kEndpoint ||
               kind == kec_car::NodeKind::kComplete)?true:false;
       //
   }
   */
    bool straight_judgment(std::string start_node_kind, std::string end_node_kind) {
            kec_car::NodeKind start_kind = car_mode_determine(start_node_kind);
            kec_car::NodeKind end_kind = car_mode_determine(end_node_kind);
            // 연결-연결, 연결-완료, 연결-주행 종료, 연결-일시 정지, 교차로-연결, 교차로-완료, 교차로-주행 종료, 교차로-일시 정지,
            // 완료-연결, 완료-완료, 완료-주행 종료, 완료-일시 정지, 종점-연결 조건을 확인하여 직진 여부 판단
            if ((start_kind == kec_car::NodeKind::kConnecting && end_kind == kec_car::NodeKind::kConnecting) ||
                (start_kind == kec_car::NodeKind::kConnecting && end_kind == kec_car::NodeKind::kComplete) ||
                (start_kind == kec_car::NodeKind::kConnecting && end_kind == kec_car::NodeKind::kEndpoint) ||
                (start_kind == kec_car::NodeKind::kConnecting && end_kind == kec_car::NodeKind::kWaiting) ||
                (start_kind == kec_car::NodeKind::kIntersection && end_kind == kec_car::NodeKind::kConnecting) ||
                (start_kind == kec_car::NodeKind::kIntersection && end_kind == kec_car::NodeKind::kComplete) ||
                (start_kind == kec_car::NodeKind::kIntersection && end_kind == kec_car::NodeKind::kEndpoint) ||
                (start_kind == kec_car::NodeKind::kIntersection && end_kind == kec_car::NodeKind::kWaiting) ||
                (start_kind == kec_car::NodeKind::kComplete && end_kind == kec_car::NodeKind::kConnecting) ||
                (start_kind == kec_car::NodeKind::kComplete && end_kind == kec_car::NodeKind::kComplete) ||
                (start_kind == kec_car::NodeKind::kComplete && end_kind == kec_car::NodeKind::kEndpoint) ||
                (start_kind == kec_car::NodeKind::kComplete && end_kind == kec_car::NodeKind::kWaiting) ||
                (start_kind == kec_car::NodeKind::kEndpoint && end_kind == kec_car::NodeKind::kConnecting)) {
                    return true;
            } else {
                    return false;
            }
    }

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

};


#endif //ROUTE_TRACKER_DATA_TYPE_TRANS_HPP
