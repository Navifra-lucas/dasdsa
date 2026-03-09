#ifndef NC_ACTION_PLAYBACK_SPEED_H
#define NC_ACTION_PLAYBACK_SPEED_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionPlaybackSpeed : public ActionBase {
public:
    NcActionPlaybackSpeed();
    virtual ~NcActionPlaybackSpeed();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("playback_speed", NcActionPlaybackSpeed, ActionType::DEFAULT)
}  // namespace NaviFra
#endif