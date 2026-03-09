#ifndef NC_ACTION_EXD_STATUS_H
#define NC_ACTION_EXD_STATUS_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionEXDStatus : public ActionBase {
public:
    NcActionEXDStatus();
    virtual ~NcActionEXDStatus();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("start_exd_status", NcActionEXDStatus, ActionType::VOLVO)
}  // namespace NaviFra
#endif