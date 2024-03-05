//
// Created by nuc-bt on 24. 3. 5.
//

#include <memory>
#include "route_tracker/center.hpp"
#include "math/distance.hpp"

void Center::fn_run() {
    Distance distance;
    std::shared_ptr<GpsData> gps_a = std::make_shared<GpsData>(35.09518,128.9606
    );
    std::shared_ptr<GpsData> gps_b = std::make_shared<GpsData>(35.0852,128.8786
    );
    distance.haversine_calculate_distance(*gps_a,*gps_b);
}
