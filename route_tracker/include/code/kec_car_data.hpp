//
// Created by nuc-bt on 24. 3. 7.
//

#ifndef ROUTE_TRACKER_KEC_CAR_DATA_HPP
#define ROUTE_TRACKER_KEC_CAR_DATA_HPP

namespace kec_car{
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
    enum class ActionCode{
        kWaiting,
        kDriving,
        kStop,
        kSkip
    };
    enum class DrivingMode{
        kStraight,
        kRecovery,
        kStop,
        kParking,
        kCrossroads,
        kArrive
    };
    enum class Direction{
        kForward,
        kBackward
    };
}

#endif //ROUTE_TRACKER_KEC_CAR_DATA_HPP
