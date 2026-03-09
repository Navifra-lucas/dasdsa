#ifndef NC_ACTION_OBSTACLE_H
#define NC_ACTION_OBSTACLE_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionObstacle : public ActionBase {
public:
    NcActionObstacle();
    virtual ~NcActionObstacle();

private:
    ros::NodeHandle nh;
    ros::ServiceClient camera_cmd_req;
    ros::ServiceClient camera_roi_req;

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("obstacle", NcActionObstacle, ActionType::DEFAULT)
}  // namespace NaviFra
#endif  // NC_ACTION_OBSTACLE_H