#ifndef NC_ACTION_DOCKING_H
#define NC_ACTION_DOCKING_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionDocking : public ActionBase {
public:
    NcActionDocking();
    virtual ~NcActionDocking();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
    ///
};

REGISTER_ACTION("docking", NcActionDocking, ActionType::DEFAULT)

}  // namespace NaviFra
#endif
