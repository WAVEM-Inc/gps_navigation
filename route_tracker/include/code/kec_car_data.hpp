//
// Created by nuc-bt on 24. 3. 7.
//

#ifndef ROUTE_TRACKER_KEC_CAR_DATA_HPP
#define ROUTE_TRACKER_KEC_CAR_DATA_HPP

namespace kec_car {
    enum class Type {
        kWorkNode,
        kPathNode,
        kWorkPlaceNode
    };
    /**
     * @brief Required information for node when requesting action
     */
    enum class NodeKind {
        // 교차로 노드
        kIntersection,
        //직진 노드
        kConnecting,
        //작업 완료 노드
        kComplete,
        // 종료 노드
        kEndpoint,
        // 대기 노드
        kWaiting
    };
    /**
     * @brief Required information for node when requesting action
     */
    enum class Direction {
        kForward,
        kBackward
    };
    /**
     * @brief Required information for node when requesting action
     */
    enum class DrivingOption {
        kOdom,
        kGps
    };
    enum class ActionCode {
        kWaiting,
        kDriving,
        kStop,
        kCooperative
    };
    enum class DrivingMode {
        kStraight,
        kRecovery,
        kStop,
        kParking,
        kCrossroads,
        kArrive
    };
    enum class DrivingSpeedSetting{
        kGpsStraight,
        kGpsTurn,
        kOdomStraight,
        kOdomTurn
    };
    enum class Mission{
        kSUCCESS,
        kFAILED
    };
}

#endif //ROUTE_TRACKER_KEC_CAR_DATA_HPP
