#ifndef NC_ACTION_GLOBAL_PATH_PLANNER_H
#define NC_ACTION_GLOBAL_PATH_PLANNER_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionGlobalPathplanner : public ActionBase {
public:
    NcActionGlobalPathplanner();
    virtual ~NcActionGlobalPathplanner();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("globalplanner", NcActionGlobalPathplanner, ActionType::DEFAULT)
}  // namespace NaviFra
#endif  // NC_ACTION_GLOBAL_PATH_PLANNER_H