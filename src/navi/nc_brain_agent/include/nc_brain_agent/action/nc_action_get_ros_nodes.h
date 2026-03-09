#ifndef NC_ACTION_GET_ROS_NODES_H
#define NC_ACTION_GET_ROS_NODES_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionGetROSNodes : public ActionBase {
public:
    NcActionGetROSNodes();
    virtual ~NcActionGetROSNodes();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("get_ros_nodes", NcActionGetROSNodes, ActionType::DEFAULT)
}  // namespace NaviFra
#endif