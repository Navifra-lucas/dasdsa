#ifndef NC_ACTION_SOUND_H
#define NC_ACTION_SOUND_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionSound : public ActionBase {
public:
    NcActionSound();
    virtual ~NcActionSound();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("sound", NcActionSound, ActionType::DEFAULT)
}  // namespace NaviFra
#endif