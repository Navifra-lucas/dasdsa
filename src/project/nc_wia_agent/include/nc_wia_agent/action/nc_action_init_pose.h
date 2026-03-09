#ifndef NC_ACTION_INIT_POSE_H
#define NC_ACTION_INIT_POSE_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionInitPose : public ActionBase {
public:
    NcActionInitPose();
    virtual ~NcActionInitPose();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("initposition", NcActionInitPose, ActionType::DEFAULT)
}  // namespace NaviFra
#endif  // NC_ACTION_INIT_POSE_H