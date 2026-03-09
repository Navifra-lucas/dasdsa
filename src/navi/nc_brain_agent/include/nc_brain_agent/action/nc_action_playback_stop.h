#ifndef NC_ACTION_PLAYBACK_STOP_H
#define NC_ACTION_PLAYBACK_STOP_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionPlaybackStop : public ActionBase {
public:
    NcActionPlaybackStop();
    virtual ~NcActionPlaybackStop();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
    ///
};

REGISTER_ACTION("playback_stop", NcActionPlaybackStop, ActionType::DEFAULT)
}  // namespace NaviFra
#endif
