#ifndef NC_ACTION_PLAYBACK_PAUSE_H
#define NC_ACTION_PLAYBACK_PAUSE_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionPlaybackPause : public ActionBase {
public:
    NcActionPlaybackPause();
    virtual ~NcActionPlaybackPause();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
    ///
};

REGISTER_ACTION("playback_pause", NcActionPlaybackPause, ActionType::DEFAULT)
}  // namespace NaviFra
#endif
