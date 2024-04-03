//
// Created by nuc-bt on 24. 3. 4.
//

#ifndef ROUTE_TRACKER_DISTANCE_HPP
#define ROUTE_TRACKER_DISTANCE_HPP


#include "entity/gps_data.hpp"
#include "proj.h"
#include "geo_trans.hpp"

class Distance {
public :
    Distance();
    double distance_haversine_calculate(GpsData first, GpsData second);
    double distance_from_perpendicular_line(GpsData start_node, GpsData end_node, GpsData cur_place);
    double distance_braking_calculate(const double velocity, const double friction_coefficient, const double deceleration);
    double distance_gps_to_ktm(GpsData first, GpsData second);

private :
    GeoTrans geo_trans_;
    double degree_to_radian(const double degree);
    void convert_gps_to_ktm(GpsData& original);
};

#endif //ROUTE_TRACKER_DISTANCE_HPP
