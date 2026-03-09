#ifndef NC_ACTION_ADDTASK_H
#define NC_ACTION_ADDTASK_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionAddTask : public ActionBase {
public:
    NcActionAddTask();
    virtual ~NcActionAddTask();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("add_task", NcActionAddTask, ActionType::DEFAULT)
}  // namespace NaviFra

#endif