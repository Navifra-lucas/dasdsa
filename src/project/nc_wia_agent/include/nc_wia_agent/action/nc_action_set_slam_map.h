#ifndef NC_ACTION_SET_SLAM_MAP_H
#define NC_ACTION_SET_SLAM_MAP_H

#include <Poco/Environment.h>
#include <core_agent/action/action_base.h>
#include <ros/ros.h>

#include <filesystem>
#include <fstream>

namespace NaviFra {
class NcActionSetSlamMap : public ActionBase {
public:
    NcActionSetSlamMap();
    virtual ~NcActionSetSlamMap();
private:
    ros::NodeHandle nh;
    ros::Publisher map_pub_;

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("savemapdb", NcActionSetSlamMap, ActionType::DEFAULT)
}  // namespace NaviFra
#endif