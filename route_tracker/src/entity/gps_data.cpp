//
// Created by nuc-bt on 24. 3. 4.
//


#include "gps_data.hpp"

double GpsData::fn_get_latitude() const {
    return latitude_;
}

void GpsData::fn_set_longitude(double longitude) {
    longitude_ = longitude;
}

void GpsData::fn_set_latitude(double latitude) {
    latitude_ = latitude;
}

double GpsData::fn_get_longitude() const {
    return longitude_;
}

GpsData::GpsData(const double latitude, const double longitude) {
    this->latitude_= latitude;
    this->longitude_= longitude;
}
