#ifndef NC_ACTION_AMR_BASIC_H
#define NC_ACTION_AMR_BASIC_H

#include <core_agent/action/action_base.h>
#include <std_msgs/Byte.h>

namespace NaviFra {
class NcActionAlarmVolume : public ActionBase {
public:
    NcActionAlarmVolume();
    virtual ~NcActionAlarmVolume();

private:
    ros::NodeHandle nh;
    ros::Publisher send_volume_;

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("alarm_volume", NcActionAlarmVolume, ActionType::DEFAULT)
}  // namespace NaviFra
#endif  // NC_ACTION_AMR_BASIC_H