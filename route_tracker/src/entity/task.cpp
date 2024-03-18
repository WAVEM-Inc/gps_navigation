//
// Created by nuc-bt on 24. 3. 18.
//

#include "task.hpp"


TaskGoal::TaskGoal(TaskGoal::Node cur_node, TaskGoal::Node next_node) {
    cur_node_= cur_node;
    next_node_= next_node;
}

kec_car::NodeKind TaskGoal::get_cur_node_kind() const {
    return trans_.string_to_node_kind(cur_node_.kind);
}

kec_car::NodeKind TaskGoal::get_next_node_kind() const {
    return trans_.string_to_node_kind(next_node_.kind);
}


