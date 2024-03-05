//
// Created by nuc-bt on 24. 3. 4.
//

#include "math/distance.hpp"
#include <math.h>
#include "rclcpp/rclcpp.hpp"
// latitude and longitude degree

/**
 *
 * @param first
 * @param second
 * @return
 * @brief
    *
    Haversine formula: a = sin²(Δφ/2) + cos φ1 ⋅ cos φ2 ⋅ sin²(Δλ/2)
    c = 2 ⋅* atan2( √a, √(1−a) )
    d = R ⋅* c
    where	φ is latitude, λ is longitude, R is earth’s radius (mean radius = 6,371km);
    note that angles need to be in radians to pass to trig functions!
    JavaScript:
    const R = 6371e3; // metres
    const φ1 = lat1 * Math.PI/180; // φ, λ in radians
    const φ2 = lat2 * Math.PI/180;
    const Δφ = (lat2-lat1) * Math.PI/180;
    const Δλ = (lon2-lon1) * Math.PI/180;

    const a = Math.sin(Δφ/2) * Math.sin(Δφ/2) +
              Math.cos(φ1) * Math.cos(φ2) *
              Math.sin(Δλ/2) * Math.sin(Δλ/2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
    const d = R * c; // in metres
    * use guide
    Distance distance;
    std::shared_ptr<GpsData> gps_a = std::make_shared<GpsData>(35.09518,128.9606
    );
    std::shared_ptr<GpsData> gps_b = std::make_shared<GpsData>(35.0852,128.8786
    );
    distance.haversine_calculate_distance(*gps_a,*gps_b);
*/
double Distance::haversine_calculate_distance(GpsData first,GpsData second) {
    const double earth_radius = 6371.0;
    const double d_lat = degree_to_radian(second.fn_get_latitude() - first.fn_get_latitude());
    const double d_lon = degree_to_radian(second.fn_get_longitude() - first.fn_get_longitude());
    // latitude and longitude degree change
    const double convert_latitude_first= degree_to_radian(first.fn_get_latitude());
    const double convert_latitude_second = degree_to_radian(second.fn_get_latitude());

    const double a = std::pow(std::sin(d_lat/2), 2)
            + std::cos(convert_latitude_first)
            * std::cos(convert_latitude_second)
            * std::pow(std::sin(d_lon/2), 2);

    std::cout <<std::sqrt(a)<< " "<<sqrt(1-a)<<" "<<std::atan2(std::sqrt(a),sqrt(1-a));
    const double c = 2* std::atan2(std::sqrt(a),sqrt(1-a));
    std::cout << "[RouteTracker] Distance , haversine_calculate_distance LINE : "<<__LINE__<<" "<<
    first.fn_get_latitude()<<" " << first.fn_get_longitude() <<","<< second.fn_get_latitude()<<" "<<second.fn_get_longitude()<<" distance : "<<earth_radius * c<<std::endl;
    return earth_radius * c;
}

double Distance::degree_to_radian(const double degree) {
    return degree * (M_PI / 180.0);
}