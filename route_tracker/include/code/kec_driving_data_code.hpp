//
// Created by nuc-bt on 24. 3. 11.
//

#ifndef ROUTE_TRACKER_KEC_DRIVING_DATA_CODE_HPP
#define ROUTE_TRACKER_KEC_DRIVING_DATA_CODE_HPP

namespace kec_driving_code{
    enum class Result{
        kSuccess=1001,
        kFailedCancel=2001,
        kFailedErrorRoute=2002,
        kFailedOverWaiting=2003,
        kFailedSystemError=2004,
        kAborted=3001
    };
    enum class FeedBack{
        kStart = 1001,
        kWorking=2001,
        kLidar=3001,
        kCooperation=3002,
        kSensorWaiting=4001,
        kCancel=5001
    };
}

#endif //ROUTE_TRACKER_KEC_DRIVING_DATA_CODE_HPP
