#ifndef NC_ACTION_INIT_POSE_VOLVO_H
#define NC_ACTION_INIT_POSE_VOLVO_H

#include <nc_brain_agent/action/nc_action_initpose.h>

namespace NaviFra {
class NcActionInitPoseVolvo : public NcActionInitPose {
public:
    NcActionInitPoseVolvo();
    virtual ~NcActionInitPoseVolvo();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("init_pose", NcActionInitPoseVolvo, ActionType::VOLVO)
}  // namespace NaviFra
#endif