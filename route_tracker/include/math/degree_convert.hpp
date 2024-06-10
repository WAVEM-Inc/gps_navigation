//
// Created by ubuntu on 24. 6. 9.
//

#ifndef ROUTE_TRACKER_DEGREE_CONVERT_HPP
#define ROUTE_TRACKER_DEGREE_CONVERT_HPP

#include <string>
#include <iostream>
#include <utility>

class DegreeConvert {
public:
    std::tuple<int, double> parse_input(double input);
    double convert_fraction(double fraction);
private :
    int extract_fractional_part(double input);
};


#endif //ROUTE_TRACKER_DEGREE_CONVERT_HPP
