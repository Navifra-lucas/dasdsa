#ifndef NC_ACTION_LIFT_STATUS_H
#define NC_ACTION_LIFT_STATUS_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionLiftStatus : public ActionBase {
public:
    NcActionLiftStatus();
    virtual ~NcActionLiftStatus();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("lift_status", NcActionLiftStatus, ActionType::DEFAULT)
}  // namespace NaviFra
#endif  // NC_ACTION_LIFT_STATUS_H