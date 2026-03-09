#ifndef NC_ACTION_STOP_SLAM_VOLVO_H
#define NC_ACTION_STOP_SLAM_VOLVO_H

#include <nc_brain_agent/action/nc_action_stop_slam.h>

namespace NaviFra {
class NcActionStopSLAMVolvo : public NcActionStopSLAM {
public:
    NcActionStopSLAMVolvo();
    virtual ~NcActionStopSLAMVolvo();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("stop_slam", NcActionStopSLAMVolvo, ActionType::VOLVO)
}  // namespace NaviFra
#endif