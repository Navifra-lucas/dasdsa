#ifndef NC_ACTION_AMR_VEL_H
#define NC_ACTION_AMR_VEL_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionAMRVel : public ActionBase {
public:
    NcActionAMRVel();
    virtual ~NcActionAMRVel();

private:
    ros::NodeHandle nh;
    ros::Publisher max_speed_pub_;

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("amr_vel", NcActionAMRVel, ActionType::DEFAULT)
}  // namespace NaviFra
#endif  // NC_ACTION_AMR_VEL_H