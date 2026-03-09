#ifndef NC_ACTION_FORKWIDECMD_H
#define NC_ACTION_FORKWIDECMD_H

#include "nc_wia_agent/data/robot_basic_status.h"
#include <core_msgs/WiaForkInfo.h>

#include <core_agent/action/action_base.h>
#include <ros/ros.h>

enum ForkWideStatus
{
    IDLE,
    VALUE_ERROR,
    UNKNOWN_ERROR
};


namespace NaviFra {
class NcActionForkWideCmd : public ActionBase {
public:
    NcActionForkWideCmd();
    virtual ~NcActionForkWideCmd();

private:
    ForkWideStatus checkForkWideStatus(int height);
    ros::NodeHandle nh_;
    ros::Publisher pub_fork_;


protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("fork_wide_cmd", NcActionForkWideCmd, ActionType::DEFAULT)
}  // namespace NaviFra
#endif  // NC_ACTION_FORKTILTCMD_H