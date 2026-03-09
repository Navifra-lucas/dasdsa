#ifndef NC_ACTION_OBSTACLE_DATA_H
#define NC_ACTION_OBSTACLE_DATA_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionObstacleData : public ActionBase {
public:
    NcActionObstacleData();
    virtual ~NcActionObstacleData();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("obstacle_data", NcActionObstacleData, ActionType::DEFAULT)
}  // namespace NaviFra
#endif  // NC_ACTION_OBSTACLE_DATA_H