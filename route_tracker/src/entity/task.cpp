//
// Created by nuc-bt on 24. 3. 18.
//

#include "task.hpp"


TaskGoal::TaskGoal(TaskGoal::Node cur_node, TaskGoal::Node next_node) : cur_node_(cur_node), next_node_(next_node), rotation_straight_check_(false){
}

kec_car::NodeKind TaskGoal::get_cur_node_kind() {
    return trans_.car_mode_determine(cur_node_.kind);
}

kec_car::NodeKind TaskGoal::get_next_node_kind() {
    return trans_.car_mode_determine(next_node_.kind);
}

double TaskGoal::get_cur_heading() {
        return cur_node_.heading;
}

GpsData TaskGoal::get_cur_gps() {
        return GpsData(cur_node_.position.latitude,cur_node_.position.longitude);
}

GpsData TaskGoal::get_next_gps() {
        return GpsData(next_node_.position.longitude,next_node_.position.latitude);
}

double TaskGoal::get_next_heading() {
        return next_node_.heading;
}


