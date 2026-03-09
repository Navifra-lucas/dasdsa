#ifndef NC_ACTION_PLAYBACK_PLAY_H
#define NC_ACTION_PLAYBACK_PLAY_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionPlaybackPlay : public ActionBase {
public:
    NcActionPlaybackPlay();
    virtual ~NcActionPlaybackPlay();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
    ///
};

REGISTER_ACTION("playback_play", NcActionPlaybackPlay, ActionType::DEFAULT)
}  // namespace NaviFra
#endif
