#ifndef NC_ACTION_CMD_VEL_VOLVO_H
#define NC_ACTION_CMD_VEL_VOLVO_H

#include <nc_brain_agent/action/nc_action_cmdvel.h>

namespace NaviFra {
class NcActionCMDVelVolvo : public NcActionCMDVel {
public:
    NcActionCMDVelVolvo();
    virtual ~NcActionCMDVelVolvo();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("control", NcActionCMDVelVolvo, ActionType::VOLVO)
}  // namespace NaviFra
#endif