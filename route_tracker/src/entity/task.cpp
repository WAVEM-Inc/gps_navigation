//
// Created by nuc-bt on 24. 3. 18.
//

#include "task.hpp"


Task::Task(Task::Node cur_node, Task::Node next_node) {
    cur_node_= cur_node;
    next_node_= next_node;
}

kec_car::NodeKind Task::get_cur_node_kind() const {
    return trans_.string_to_node_kind(cur_node_.kind);
}

