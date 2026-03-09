#ifndef NC_ACTION_PLAYBACK_SEEK_H
#define NC_ACTION_PLAYBACK_SEEK_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionPlaybackSeek : public ActionBase {
public:
    NcActionPlaybackSeek();
    virtual ~NcActionPlaybackSeek();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("playback_seek", NcActionPlaybackSeek, ActionType::DEFAULT)
}  // namespace NaviFra
#endif