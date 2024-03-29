//
// Created by nuc-bt on 24. 3. 7.
//

#include "car.hpp"

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

Car::Car() : friction_coefficient_(FRICTION_COEFFICIENT),
             deceleration_(DECELERATION),
             drive_mode_(kec_car::DrivingMode::kStop),
             cur_node_kind_(kec_car::NodeKind::kWaiting){
}

double Car::get_friction_coefficient() const {
    return friction_coefficient_;
}

void Car::set_friction_coefficient(double friction_coefficient) {
    friction_coefficient_ = friction_coefficient;
}

double Car::get_deceleration() const {
    return deceleration_;
}

void Car::set_deceleration(double deceleration) {
    deceleration_ = deceleration;
}

kec_car::DrivingMode Car::get_drive_mode() const {
    return drive_mode_;
}

void Car::set_drive_mode(kec_car::DrivingMode drive_mode) {
    drive_mode_ = drive_mode;
}

double Car::get_speed() const {
        return speed_;
}

void Car::set_speed(double speed) {
        speed_ = speed;
}
