#ifndef NC_ACTION_LIFT_CMD_ORIGIN_H
#define NC_ACTION_LIFT_CMD_ORIGIN_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionLiftOrigin : public ActionBase {
public:
    NcActionLiftOrigin();
    virtual ~NcActionLiftOrigin();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("lift_cmd_origin", NcActionLiftOrigin, ActionType::DEFAULT)
}  // namespace NaviFra
#endif  // NC_ACTION_LIFT_CMD_ORIGIN_H