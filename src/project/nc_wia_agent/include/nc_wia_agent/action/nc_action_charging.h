#ifndef NC_ACTION_CHARING_H
#define NC_ACTION_CHARING_H

#include "nc_wia_agent/data/robot_basic_status.h"

#include <core_agent/action/action_base.h>
#include <ros/ros.h>

namespace NaviFra {
class NcActionCharging : public ActionBase {
public:
    NcActionCharging();
    virtual ~NcActionCharging();

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_charging_;


protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("charging", NcActionCharging, ActionType::DEFAULT)
}  // namespace NaviFra
#endif  // NC_ACTION_FORKTILTCMD_H