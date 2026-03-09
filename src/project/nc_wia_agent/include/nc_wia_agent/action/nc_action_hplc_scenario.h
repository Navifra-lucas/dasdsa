#ifndef NC_ACTION_PLC_SCENARIO_H
#define NC_ACTION_PLC_SCENARIO_H

#include "nc_wia_agent/data/robot_basic_status.h"
#include "nc_wia_agent/nc_wia_agent.h"

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionPlcScenario : public ActionBase {
public:
    NcActionPlcScenario();
    virtual ~NcActionPlcScenario();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("hplc_scenario", NcActionPlcScenario, ActionType::DEFAULT)
}  // namespace NaviFra
#endif  // NC_ACTION_PLC_SCENARIO_H