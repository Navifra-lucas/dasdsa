#ifndef NC_ACTION_PLAYBACK_DOWNLOAD_H
#define NC_ACTION_PLAYBACK_DOWNLOAD_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionPlaybackDownload : public ActionBase {
public:
    NcActionPlaybackDownload();
    virtual ~NcActionPlaybackDownload();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("playback_download", NcActionPlaybackDownload, ActionType::DEFAULT)
}  // namespace NaviFra
#endif