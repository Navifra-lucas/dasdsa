#ifndef NC_ACTION_RESULT_H
#define NC_ACTION_RESULT_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionResult : public ActionBase {
public:
    NcActionResult();
    virtual ~NcActionResult();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("result", NcActionResult, ActionType::DEFAULT)
}  // namespace NaviFra
#endif  // NC_ACTION_RESULT_H