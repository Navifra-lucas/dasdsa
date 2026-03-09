#ifndef NC_ACTION_START_SCAN_VOLVO_H
#define NC_ACTION_START_SCAN_VOLVO_H

#include <nc_brain_agent/action/nc_action_start_scan.h>

namespace NaviFra {
class NcActionStartSCANVolvo : public NcActionStartSCAN {
public:
    NcActionStartSCANVolvo();
    virtual ~NcActionStartSCANVolvo();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("start_scan", NcActionStartSCANVolvo, ActionType::VOLVO)
}  // namespace NaviFra
#endif