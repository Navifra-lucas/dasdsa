#ifndef NC_ACTION_RESET_MILEAGE_H
#define NC_ACTION_RESET_MILEAGE_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionResetMileage : public ActionBase {
public:
    NcActionResetMileage();
    virtual ~NcActionResetMileage();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("reset_mileage", NcActionResetMileage, ActionType::DEFAULT)
}  // namespace NaviFra
#endif  // NC_ACTION_RESET_MILEAGE_H