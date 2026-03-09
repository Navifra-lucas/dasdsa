#ifndef NC_ACTION_PLC_OFFSET_H
#define NC_ACTION_PLC_OFFSET_H

#include "nc_wia_agent/data/robot_basic_status.h"
#include "nc_wia_agent/nc_wia_agent.h"

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionPlcData : public ActionBase {
public:
    NcActionPlcData();
    virtual ~NcActionPlcData();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("hplc_offset", NcActionPlcData, ActionType::DEFAULT)
}  // namespace NaviFra
#endif  // NC_ACTION_PLC_OFFSET_H