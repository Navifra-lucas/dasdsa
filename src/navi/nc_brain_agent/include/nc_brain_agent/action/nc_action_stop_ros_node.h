#ifndef NC_ACTION_STOP_ROS_NODE_H
#define NC_ACTION_STOP_ROS_NODE_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionStopROSNode : public ActionBase {
public:
    NcActionStopROSNode();
    virtual ~NcActionStopROSNode();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("stop_ros_node", NcActionStopROSNode, ActionType::DEFAULT)
}  // namespace NaviFra
#endif