#ifndef NC_ACTION_PAUSE_H
#define NC_ACTION_PAUSE_H

#include "nc_wia_agent/data/robot_basic_status.h"

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionPause : public ActionBase {
public:
    NcActionPause();
    virtual ~NcActionPause();

private:
    ros::NodeHandle nh;
    ros::Publisher task_cmd_pub_;

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("pause", NcActionPause, ActionType::DEFAULT)
}  // namespace NaviFra
#endif  // NC_ACTION_PAUSE_H