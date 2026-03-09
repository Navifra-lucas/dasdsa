#ifndef NC_ACTION_CANCEL_H
#define NC_ACTION_CANCEL_H

#include "nc_wia_agent/data/robot_basic_status.h"

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionCancel : public ActionBase {
public:
    NcActionCancel();
    virtual ~NcActionCancel();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("cancel", NcActionCancel, ActionType::DEFAULT)
}  // namespace NaviFra
#endif  // NC_ACTION_AMR_BASIC_H