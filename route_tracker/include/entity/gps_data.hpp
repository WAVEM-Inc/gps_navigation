//
// Created by nuc-bt on 24. 3. 4.
//

#ifndef ROUTE_TRACKER_GPS_DATA_HPP
#define ROUTE_TRACKER_GPS_DATA_HPP
class GpsData {
private :
    double latitude_;
    double longitude_;
public:
    GpsData(const double latitude, const double longitude);
    void fn_set_longitude(double longitude);
    void fn_set_latitude(double latitude);
    double fn_get_latitude() const;
    double fn_get_longitude() const;
};
#endif //ROUTE_TRACKER_GPS_DATA_HPP
