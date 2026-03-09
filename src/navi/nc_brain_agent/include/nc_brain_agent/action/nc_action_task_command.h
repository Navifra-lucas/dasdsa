#ifndef NC_ACTION_TASK_COMMAND_H
#define NC_ACTION_TASK_COMMAND_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionTaskCommand : public ActionBase {
public:
    NcActionTaskCommand();
    virtual ~NcActionTaskCommand();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("task_cmd", NcActionTaskCommand, ActionType::DEFAULT)
}  // namespace NaviFra

#endif