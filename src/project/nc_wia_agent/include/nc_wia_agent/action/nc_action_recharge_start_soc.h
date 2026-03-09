#ifndef NC_ACTION_RECHARGE_START_SOC_H
#define NC_ACTION_RECHARGE_START_SOC_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionRechargeStartSOC : public ActionBase {
public:
    NcActionRechargeStartSOC();
    virtual ~NcActionRechargeStartSOC();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("recharge_start_soc", NcActionRechargeStartSOC, ActionType::DEFAULT)
}  // namespace NaviFra
#endif  // NC_ACTION_RECHARGE_START_SOC_H