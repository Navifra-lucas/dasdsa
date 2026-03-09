#ifndef NC_ACTION_UPDATE_MAP_H
#define NC_ACTION_UPDATE_MAP_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionUpdateMap : public ActionBase {
public:
    NcActionUpdateMap();
    virtual ~NcActionUpdateMap();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("update_map", NcActionUpdateMap, ActionType::DEFAULT)
}  // namespace NaviFra
#endif