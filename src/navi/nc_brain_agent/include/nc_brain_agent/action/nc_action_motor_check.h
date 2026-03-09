#ifndef NC_ACTION_MOTOR_CHECK_H
#define NC_ACTION_MOTOR_CHECK_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionMotorCheck : public ActionBase {
public:
    NcActionMotorCheck();
    virtual ~NcActionMotorCheck();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("start_motorcheck", NcActionMotorCheck, ActionType::DEFAULT)
}  // namespace NaviFra
#endif