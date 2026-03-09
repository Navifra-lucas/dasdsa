#ifndef NC_ACTION_BATTERY_H
#define NC_ACTION_BATTERY_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionBattery : public ActionBase {
public:
    NcActionBattery();
    virtual ~NcActionBattery();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("battery", NcActionBattery, ActionType::DEFAULT)
}  // namespace NaviFra
#endif  // NC_ACTION_AMR_BASIC_H