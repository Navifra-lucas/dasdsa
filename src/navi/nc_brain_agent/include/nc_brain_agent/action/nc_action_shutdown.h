#ifndef NC_ACTION_SHUTDOWN_H
#define NC_ACTION_SHUTDOWN_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionShutdown : public ActionBase {
public:
    NcActionShutdown();
    virtual ~NcActionShutdown();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("shutdown", NcActionShutdown, ActionType::DEFAULT)
}  // namespace NaviFra
#endif