#ifndef NC_ACTION_STOP_SLAM_H
#define NC_ACTION_STOP_SLAM_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionStopSLAM : public ActionBase {
public:
    NcActionStopSLAM();
    virtual ~NcActionStopSLAM();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("stop_slam", NcActionStopSLAM, ActionType::DEFAULT)
}  // namespace NaviFra
#endif