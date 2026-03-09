#ifndef NAVIFRA_NODE_PATH_H
#define NAVIFRA_NODE_PATH_H

#include <string>

namespace NaviFra {
struct NodePath {
    std::string now_node;
    std::string next_node;
    std::string goal_node;
    double path_progress;
};
}  // namespace NaviFra
#endif
