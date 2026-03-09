#ifndef NC_ACTION_TASK_ALARM_H
#define NC_ACTION_TASK_ALARM_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionTaskAlarm : public ActionBase {
public:
    NcActionTaskAlarm();
    virtual ~NcActionTaskAlarm();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

}  // namespace NaviFra
#endif