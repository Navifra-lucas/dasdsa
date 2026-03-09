#ifndef NC_ACTION_LED_LIGHT_H
#define NC_ACTION_LED_LIGHT_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionLedLight : public ActionBase {
public:
    NcActionLedLight();
    virtual ~NcActionLedLight();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("led_light", NcActionLedLight, ActionType::DEFAULT)
}  // namespace NaviFra
#endif