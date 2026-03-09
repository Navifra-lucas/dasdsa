#ifndef NC_ACTION_SYNC_MAP_H
#define NC_ACTION_SYNC_MAP_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionSyncMap : public ActionBase {
public:
    NcActionSyncMap();
    virtual ~NcActionSyncMap();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("sync_map", NcActionSyncMap, ActionType::DEFAULT)
}  // namespace NaviFra
#endif