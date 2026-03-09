#ifndef NC_ACTION_LOG_DOWNLOAD_H
#define NC_ACTION_LOG_DOWNLOAD_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionLogDownload : public ActionBase {
public:
    NcActionLogDownload();
    virtual ~NcActionLogDownload();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("log_download", NcActionLogDownload, ActionType::DEFAULT)
}  // namespace NaviFra
#endif