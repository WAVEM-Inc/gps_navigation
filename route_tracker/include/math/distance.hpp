//
// Created by nuc-bt on 24. 3. 4.
//

#ifndef ROUTE_TRACKER_DISTANCE_HPP
#define ROUTE_TRACKER_DISTANCE_HPP


#include "entity/gps_data.hpp"

class Distance {
    public :
        double haversine_calculate_distance(GpsData first, GpsData second);
private :
        double degree_to_radian(const double degree);
};

#endif //ROUTE_TRACKER_DISTANCE_HPP
