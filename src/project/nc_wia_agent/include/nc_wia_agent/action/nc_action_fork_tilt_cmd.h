#ifndef NC_ACTION_FORKTILTCMD_H
#define NC_ACTION_FORKTILTCMD_H

#include "nc_wia_agent/data/robot_basic_status.h"
#include <core_msgs/WiaForkInfo.h>

#include <core_agent/action/action_base.h>
#include <ros/ros.h>

enum TiltStatus
{
    IDLE,
    VALUE_ERROR,
    UNKNOWN_ERROR
};

namespace NaviFra {
class NcActionForkTiltCmd : public ActionBase {
public:
    NcActionForkTiltCmd();
    virtual ~NcActionForkTiltCmd();

private:
    TiltStatus checkTiltStatus(float angle);
    ros::NodeHandle nh_;
    ros::Publisher pub_fork_;

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("fork_tilt_cmd", NcActionForkTiltCmd, ActionType::DEFAULT)
}  // namespace NaviFra
#endif  // NC_ACTION_FORKTILTCMD_H