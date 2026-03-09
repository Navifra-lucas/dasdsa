#ifndef NC_ACTION_DEVICE_COMMAND_H
#define NC_ACTION_DEVICE_COMMAND_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionDeviceCommand : public ActionBase {
public:
    NcActionDeviceCommand();
    virtual ~NcActionDeviceCommand();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("device_command", NcActionDeviceCommand, ActionType::DEFAULT)
}  // namespace NaviFra
#endif