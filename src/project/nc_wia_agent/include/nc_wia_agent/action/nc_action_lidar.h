#ifndef NC_ACTION_LIDAR_H
#define NC_ACTION_LIDAR_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionLidar : public ActionBase {
public:
    NcActionLidar();
    virtual ~NcActionLidar();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("lidar", NcActionLidar, ActionType::DEFAULT)
}  // namespace NaviFra
#endif  // NC_ACTION_LIDAR_H