//
// Created by nuc-bt on 24. 3. 7.
//

#ifndef ROUTE_TRACKER_CAR_HPP
#define ROUTE_TRACKER_CAR_HPP


#include "code/kec_car_data.hpp"
#include "gps_data.hpp"

class Car {
private:
    kec_car::NodeKind node_kind_;
    double degree_;
    GpsData location_;

public :
    Car();

    kec_car::NodeKind get_node_kind() const;
    void set_node_kind(kec_car::NodeKind node_kind);
    double get_degree() const;
    void set_degree(double degree);
    const GpsData get_location() const;
    void set_location(const GpsData location);
};


#endif //ROUTE_TRACKER_CAR_HPP
