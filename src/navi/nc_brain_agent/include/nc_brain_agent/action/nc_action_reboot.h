#ifndef NC_ACTION_REBOOT_H
#define NC_ACTION_REBOOT_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionReboot : public ActionBase {
public:
    NcActionReboot();
    virtual ~NcActionReboot();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("reboot", NcActionReboot, ActionType::DEFAULT)
}  // namespace NaviFra
#endif