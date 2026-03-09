#ifndef NC_ACTION_LIFT_CMD_H
#define NC_ACTION_LIFT_CMD_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionLiftCMD : public ActionBase {
public:
    NcActionLiftCMD();
    virtual ~NcActionLiftCMD();

private:
    ros::NodeHandle nh;
    ros::Publisher send_loading_;

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("lift_cmd", NcActionLiftCMD, ActionType::DEFAULT)
}  // namespace NaviFra
#endif  // NC_ACTION_LIFT_CMD_H