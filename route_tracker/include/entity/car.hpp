//
// Created by nuc-bt on 24. 3. 7.
//

#ifndef ROUTE_TRACKER_CAR_HPP
#define ROUTE_TRACKER_CAR_HPP


#include "kec_car_data.hpp"

class Car {
private:
    kec_car::NodeKind node_kind_;
    double degree_;
public :
    kec_car::NodeKind get_node_kind() const;
    void set_node_kind(kec_car::NodeKind node_kind);
    double get_degree() const;
    void set_degree(double degree);
};


#endif //ROUTE_TRACKER_CAR_HPP
