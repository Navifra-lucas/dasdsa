#ifndef NC_ACTION_SAVE_SLAM_H
#define NC_ACTION_SAVE_SLAM_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionSaveSLAM : public ActionBase {
public:
    NcActionSaveSLAM();
    virtual ~NcActionSaveSLAM();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("save_slam", NcActionSaveSLAM, ActionType::DEFAULT)
}  // namespace NaviFra
#endif