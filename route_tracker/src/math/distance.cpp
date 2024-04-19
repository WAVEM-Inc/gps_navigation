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
    distance.distance_haversine_calculate(*gps_a,*gps_b);
*/
double Distance::distance_haversine_calculate(GpsData first, GpsData second) {
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
    std::cout << "[RouteTracker] Distance , distance_haversine_calculate LINE : "<<__LINE__<<" "<<
    first.fn_get_latitude()<<" " << first.fn_get_longitude() <<","<< second.fn_get_latitude()<<" "<<second.fn_get_longitude()<<" distance : "<<earth_radius * c<<std::endl;
    //km/h -> m/sec 변환 필요
    return earth_radius * c;
}

double Distance::degree_to_radian(const double degree) {
    return degree * (M_PI / 180.0);
}

//double Distance::distance_from_perpendicular_line(double x1, double y1, double x2, double y2, double x0, double y0) {
double Distance::distance_from_perpendicular_line(GpsData start_node, GpsData end_node, GpsData cur_place) {
        double distance=0.0;
        convert_gps_to_ktm(start_node);
        convert_gps_to_ktm(end_node);
        convert_gps_to_ktm(cur_place);
        //   if (y2 - y1) == 0:
        if (end_node.fn_get_longitude() - start_node.fn_get_longitude() == 0) {
            //distance = distanceBetween(x2, y0, x0, y0)
            std::unique_ptr<GpsData> temp_first= std::make_unique<GpsData>(end_node.fn_get_latitude(),cur_place.fn_get_longitude());
            distance = distance_haversine_calculate(std::move(*temp_first), cur_place);
        }
        //elif (x2 - x1) == 0:
        else if (end_node.fn_get_latitude() - start_node.fn_get_latitude() == 0) {
            std::unique_ptr<GpsData> temp_first= std::make_unique<GpsData>(cur_place.fn_get_latitude(),end_node.fn_get_longitude());
            //distance = distanceBetween(x0, y2, x0, y0)
            distance = distance_haversine_calculate(std::move(*temp_first), cur_place);
        } else {
            // 시작 점을 지나는 직선의 기울기는
            double perpendicular_m = (end_node.fn_get_latitude() - start_node.fn_get_latitude()) / (end_node.fn_get_longitude() - start_node.fn_get_longitude()) * -1;
            // 시작 점을 지나는 직선의 방정식은 y - y1 = perpendicular_m * (x - x1)
            // 이 직선과 외부 점 사이의 거리를 계산
            distance = std::abs(
                    (perpendicular_m * cur_place.fn_get_latitude() - cur_place.fn_get_longitude() + end_node.fn_get_longitude() - perpendicular_m * end_node.fn_get_latitude())
                    / std::sqrt(1 + perpendicular_m * perpendicular_m)
                    );
        }
        return distance;
    }

Distance::Distance() {}

//제동거리=  ((V^2-V_0^2))/(2a제동효율)
// braking_distance = (0*0-velocity*velocity)/2*deceleration*friction_coefficient
double Distance::distance_braking_calculate(const double velocity,
                                            const double friction_coefficient,
                                            const double deceleration) {
    return static_cast<double>((0*0-velocity*velocity)/2*deceleration*friction_coefficient);
}
/**
 * @brief distance calculation by Korea TM
 * @param first
 * @param second
 * @return
 */
double Distance::distance_gps_to_ktm(GpsData first, GpsData second) {
    //convert_gps_to_ktm(first);
    //convert_gps_to_ktm(second);

    return 0;
}


void Distance::convert_gps_to_ktm(GpsData& original) {
    GeoPoint in_pt;
    in_pt.y = original.fn_get_latitude();
    in_pt.x= original.fn_get_longitude();
    in_pt=geo_trans_.convert(geo_trans_.GEO,geo_trans_.TM,in_pt);
    original.fn_set_latitude(in_pt.x);
    original.fn_set_longitude(in_pt.y);
    //return GpsData();
}

double Distance::calculate_line_angle(GpsData cur_place, GpsData end_node) {
    convert_gps_to_ktm(cur_place);
    convert_gps_to_ktm(end_node);
    double delta_x = end_node.fn_get_latitude() - cur_place.fn_get_latitude();
    double delta_y = end_node.fn_get_longitude() - cur_place.fn_get_longitude();
    double angle_rad = atan2(delta_y, delta_x);
    double angle_deg = radian_to_degree(angle_rad) - 90;
    angle_deg = fmod(angle_deg + 360.0, 360.0);
    return angle_deg;
}

double Distance::radian_to_degree(const double radian) {
    return radian * (180.0 / M_PI);;
}
