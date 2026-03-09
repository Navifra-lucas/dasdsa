#ifndef NC_ACTION_GET_SLAM_MAP_H
#define NC_ACTION_GET_SLAM_MAP_H

#include <Poco/Environment.h>
#include <core_agent/action/action_base.h>

#include <fstream>

namespace NaviFra {
class NcActionGetSlamMap : public ActionBase {
public:
    NcActionGetSlamMap();
    virtual ~NcActionGetSlamMap();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("loadmapdb", NcActionGetSlamMap, ActionType::DEFAULT)
// REGISTER_ACTION(NcHacsCommand::HACS_SET_SLAM_MAP, NcActionGetSlamMap, ActionType::DEFAULT)
}  // namespace NaviFra

#endif