#ifndef NC_ACTION_DEVICE_COMMAND_VOLVO_H
#define NC_ACTION_DEVICE_COMMAND_VOLVO_H

#include <nc_brain_agent/action/nc_action_device_command.h>

namespace NaviFra {
class NcActionDeviceCommandVolvo : public NcActionDeviceCommand {
public:
    NcActionDeviceCommandVolvo();
    virtual ~NcActionDeviceCommandVolvo();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("device_command", NcActionDeviceCommandVolvo, ActionType::VOLVO)
}  // namespace NaviFra
#endif