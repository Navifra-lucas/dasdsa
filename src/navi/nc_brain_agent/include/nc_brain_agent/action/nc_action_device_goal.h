#ifndef NC_ACTION_DEVICE_GOAL_H
#define NC_ACTION_DEVICE_GOAL_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionDeviceGoal : public ActionBase {
public:
    NcActionDeviceGoal();
    virtual ~NcActionDeviceGoal();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("device_goal", NcActionDeviceGoal, ActionType::DEFAULT)
}  // namespace NaviFra
#endif