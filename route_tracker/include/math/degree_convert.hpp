//
// Created by ubuntu on 24. 6. 9.
//

#ifndef ROUTE_TRACKER_DEGREE_CONVERT_HPP
#define ROUTE_TRACKER_DEGREE_CONVERT_HPP

#include <string>
#include <iostream>
#include <utility>

class DegreeConvert {
    std::pair<int, double> parse_input(const std::string& str_input);
    double convert_fraction(double fraction);
};


#endif //ROUTE_TRACKER_DEGREE_CONVERT_HPP
