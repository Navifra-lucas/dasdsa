#ifndef NC_ACTION_DEVICE_GOAL_VOLVO_H
#define NC_ACTION_DEVICE_GOAL_VOLVO_H

#include <nc_brain_agent/action/nc_action_device_goal.h>

namespace NaviFra {
class NcActionDeviceGoalVolvo : public NcActionDeviceGoal {
public:
    NcActionDeviceGoalVolvo();
    virtual ~NcActionDeviceGoalVolvo();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("device_goal", NcActionDeviceGoalVolvo, ActionType::VOLVO)
}  // namespace NaviFra
#endif