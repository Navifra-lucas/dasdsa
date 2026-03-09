#ifndef NC_ACTION_WAKEUP_H
#define NC_ACTION_WAKEUP_H

#include "nc_wia_agent/data/robot_basic_status.h"

#include <core_agent/action/action_base.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
namespace NaviFra {
class NcActionWakeup : public ActionBase {
public:
    NcActionWakeup();
    virtual ~NcActionWakeup();

private:
    ros::NodeHandle nh_;
    ros::Publisher wakeup_pub_;

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("wakeup", NcActionWakeup, ActionType::DEFAULT)
}  // namespace NaviFra
#endif  // NC_ACTION_WAKEUP_H