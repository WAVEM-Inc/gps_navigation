//
// Created by nuc-bt on 24. 3. 7.
//

#include "car.hpp"

kec_car::NodeKind Car::get_node_kind() const {
    return node_kind_;
}

void Car::set_node_kind(kec_car::NodeKind node_kind) {
    node_kind_ = node_kind;
}

double Car::get_degree() const {
    return degree_;
}

void Car::set_degree(double degree) {
    degree_ = degree;
}

const GpsData Car::get_location() const {
    return location_;
}

void Car::set_location(const GpsData location) {
    location_ = location;
}

Car::Car() {

}
