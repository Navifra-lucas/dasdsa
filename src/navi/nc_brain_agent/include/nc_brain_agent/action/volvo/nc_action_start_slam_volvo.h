#ifndef NC_ACTION_START_SLAM_VOLVO_H
#define NC_ACTION_START_SLAM_VOLVO_H

#include <nc_brain_agent/action/nc_action_start_slam.h>

namespace NaviFra {
class NcActionStartSLAMVolvo : public NcActionStartSLAM {
public:
    NcActionStartSLAMVolvo();
    virtual ~NcActionStartSLAMVolvo();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("start_slam", NcActionStartSLAMVolvo, ActionType::VOLVO)
}  // namespace NaviFra
#endif