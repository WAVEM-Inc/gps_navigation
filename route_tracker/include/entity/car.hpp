//
// Created by nuc-bt on 24. 3. 7.
//

#ifndef ROUTE_TRACKER_CAR_HPP
#define ROUTE_TRACKER_CAR_HPP


#include "code/kec_car_data.hpp"
#include "gps_data.hpp"
#define FRICTION_COEFFICIENT 0.8
#define DECELERATION 9.8

class Car {
private:
    kec_car::NodeKind cur_node_kind_;
    kec_car::DrivingMode drive_mode_;
    kec_car::Direction direction_;
    //
    double degree_;
    double speed_;
    GpsData location_;
    // 마찰 계수
    double friction_coefficient_;
    double deceleration_;

public :
    Car();

    double get_degree() const;
    void set_degree(double degree);
    const GpsData get_location() const;
    void set_location(const GpsData location);

    double get_friction_coefficient() const;
    void set_friction_coefficient(double friction_coefficient);
    double get_deceleration() const;
    void set_deceleration(double deceleration);
    kec_car::DrivingMode get_drive_mode() const;
    void set_drive_mode(kec_car::DrivingMode drive_mode);
    double get_speed() const;
    void set_speed(double speed);
    kec_car::Direction get_direction() const;
    void set_direction(kec_car::Direction direction);
};


#endif //ROUTE_TRACKER_CAR_HPP
