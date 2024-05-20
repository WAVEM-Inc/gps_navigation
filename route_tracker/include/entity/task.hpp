//
// Created by nuc-bt on 24. 3. 18.
//

#ifndef ROUTE_TRACKER_TASK_HPP
#define ROUTE_TRACKER_TASK_HPP


#include "code/kec_car_data.hpp"
#include "route_msgs/msg/node.hpp"
#include "common/data_type_trans.hpp"
#include "gps_data.hpp"

class TaskGoal {
private :
    using Node = route_msgs::msg::Node;
    Node cur_node_;
    Node next_node_;
    kec_car::DrivingOption cur_do_;
    kec_car::Direction cur_dir_;
    DataTypeTrans trans_;
public :
    bool rotation_straight_check_;
    TaskGoal(Node cur_node, Node next_node);
    Node bypass_cur_node_;
    Node bypass_next_node_;
    kec_car::NodeKind get_cur_node_kind();
    kec_car::NodeKind get_next_node_kind();
    GpsData get_cur_gps();
    GpsData get_next_gps();

    double get_cur_heading();
    double get_next_heading();
    kec_car::DrivingOption get_cur_driving_option();
    kec_car::Direction get_cur_dir();

};


#endif //ROUTE_TRACKER_TASK_HPP
