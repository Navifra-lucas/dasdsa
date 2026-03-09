#ifndef NC_ACTION_ROBOT_LIST_H
#define NC_ACTION_ROBOT_LIST_H

#include <core_agent/action/action_base.h>
#include <core_msgs/Vehicle.h>
#include <core_msgs/VehicleList.h>
#include "nc_wia_agent/data/robot_basic_status.h"
#include <ros/ros.h>

namespace NaviFra {
class NcActionRobotList : public ActionBase {
public:
    NcActionRobotList();
    virtual ~NcActionRobotList();
private:
    ros::NodeHandle nh;
    ros::Publisher v2v_pub_;

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("robot_list", NcActionRobotList, ActionType::DEFAULT)
}  // namespace NaviFra
#endif  // NC_ACTION_ROBOT_LIST_H