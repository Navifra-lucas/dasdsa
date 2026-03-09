#ifndef NC_ACTION_MOVE_H
#define NC_ACTION_MOVE_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionMove : public ActionBase {
public:
    NcActionMove();
    virtual ~NcActionMove();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("move", NcActionMove, ActionType::DEFAULT)
}  // namespace NaviFra
#endif