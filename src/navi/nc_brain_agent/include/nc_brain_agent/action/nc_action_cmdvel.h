#ifndef NC_ACTION_CMD_VEL_H
#define NC_ACTION_CMD_VEL_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionCMDVel : public ActionBase {
public:
    NcActionCMDVel();
    virtual ~NcActionCMDVel();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("control", NcActionCMDVel, ActionType::DEFAULT)
}  // namespace NaviFra
#endif