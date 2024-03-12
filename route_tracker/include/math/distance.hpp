//
// Created by nuc-bt on 24. 3. 4.
//

#ifndef ROUTE_TRACKER_DISTANCE_HPP
#define ROUTE_TRACKER_DISTANCE_HPP


#include "entity/gps_data.hpp"

class Distance {
public :
    Distance();
    double haversine_calculate_distance(GpsData first, GpsData second);
    double distance_from_perpendicular_line(GpsData start_node, GpsData end_node, GpsData cur_place);
    double calculate_braking_distance(const double velocity, const double friction_coefficient,const double deceleration);
private :
        double degree_to_radian(const double degree);
};

#endif //ROUTE_TRACKER_DISTANCE_HPP
