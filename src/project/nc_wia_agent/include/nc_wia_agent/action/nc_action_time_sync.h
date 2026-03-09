#ifndef NC_ACTION_TIME_SYNC_H
#define NC_ACTION_TIME_SYNC_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionTimeSync : public ActionBase {
public:
    NcActionTimeSync();
    virtual ~NcActionTimeSync();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("timesync", NcActionTimeSync, ActionType::DEFAULT)
}  // namespace NaviFra
#endif  // NC_ACTION_TIME_SYNC_H