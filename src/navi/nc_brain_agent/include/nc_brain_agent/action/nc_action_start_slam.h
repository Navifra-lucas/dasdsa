#ifndef NC_ACTION_START_SLAM_H
#define NC_ACTION_START_SLAM_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionStartSLAM : public ActionBase {
public:
    NcActionStartSLAM();
    virtual ~NcActionStartSLAM();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("start_slam", NcActionStartSLAM, ActionType::DEFAULT)
}  // namespace NaviFra
#endif