#ifndef NC_ACTION_FORKHEIGHTCMD_H
#define NC_ACTION_FORKHEIGHTCMD_H

#include "nc_wia_agent/data/robot_basic_status.h"
#include <core_msgs/WiaForkInfo.h>

#include <core_agent/action/action_base.h>
#include <ros/ros.h>

enum ForkStatus
{
    IDLE,
    MODE_ERROR,
    VALUE_ERROR,
    UNKNOWN_ERROR
};

namespace NaviFra {
class NcActionForkHeightCmd : public ActionBase {
public:
    NcActionForkHeightCmd();
    virtual ~NcActionForkHeightCmd();
    ForkStatus checkForkStatus(int mode, int height);

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_fork_;

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("fork_height_cmd", NcActionForkHeightCmd, ActionType::DEFAULT)
}  // namespace NaviFra
#endif  // NC_ACTION_FORKHEIGHTCMD_H