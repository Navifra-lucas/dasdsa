#ifndef DOCKING_NODES_HPP_
#define DOCKING_NODES_HPP_

#include "utils/pose.hpp"

namespace NVFR {

struct DockingNodes_t
{
    Pose o_fNode;
    Pose o_nNode;
    Pose o_gNode;
};

}  // namespace NVFR

#endif
