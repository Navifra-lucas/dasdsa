#ifndef NC_ACTION_START_ROS_NODE_H
#define NC_ACTION_START_ROS_NODE_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionStartROSNode : public ActionBase {
public:
    NcActionStartROSNode();
    virtual ~NcActionStartROSNode();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("start_ros_node", NcActionStartROSNode, ActionType::DEFAULT)
}  // namespace NaviFra
#endif