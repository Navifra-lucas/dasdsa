#ifndef NC_ACTION_RELAY_COUNT_H
#define NC_ACTION_RELAY_COUNT_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionRelayCount : public ActionBase {
public:
    NcActionRelayCount();
    virtual ~NcActionRelayCount();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("relay_count", NcActionRelayCount, ActionType::DEFAULT)
}  // namespace NaviFra
#endif  // NC_ACTION_RELAY_COUNT_H