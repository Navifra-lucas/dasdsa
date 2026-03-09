#ifndef NC_ACTION_ACTION_H
#define NC_ACTION_ACTION_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionAction : public ActionBase {
public:
    NcActionAction();
    virtual ~NcActionAction();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
    ///
};

REGISTER_ACTION("action", NcActionAction, ActionType::DEFAULT)

}  // namespace NaviFra
#endif
