#ifndef NC_ACTION_START_SCAN_H
#define NC_ACTION_START_SCAN_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionStartSCAN : public ActionBase {
public:
    NcActionStartSCAN();
    virtual ~NcActionStartSCAN();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("start_scan", NcActionStartSCAN, ActionType::DEFAULT)
}  // namespace NaviFra
#endif