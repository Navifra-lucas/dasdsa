#ifndef NC_ACTION_STOP_H
#define NC_ACTION_STOP_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionStop : public ActionBase {
public:
    NcActionStop();
    virtual ~NcActionStop();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("stop", NcActionStop, ActionType::DEFAULT)
}  // namespace NaviFra
#endif