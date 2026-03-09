#ifndef NC_ACTION_REPEAT_DRIVE_H
#define NC_ACTION_REPEAT_DRIVE_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionRepeatDrive : public ActionBase {
public:
    NcActionRepeatDrive();
    virtual ~NcActionRepeatDrive();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("repeat_drive", NcActionRepeatDrive, ActionType::DEFAULT)
}  // namespace NaviFra
#endif