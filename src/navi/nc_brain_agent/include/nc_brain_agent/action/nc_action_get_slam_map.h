#ifndef NC_ACTION_GET_SLAM_MAP_H
#define NC_ACTION_GET_SLAM_MAP_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionGetSLAMMap : public ActionBase {
public:
    NcActionGetSLAMMap();
    virtual ~NcActionGetSLAMMap();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("get_slam_map", NcActionGetSLAMMap, ActionType::DEFAULT)
}  // namespace NaviFra
#endif