#ifndef NC_ACTION_ERROR_RESET_H
#define NC_ACTION_ERROR_RESET_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionErrorReset : public ActionBase {
public:
    NcActionErrorReset();
    virtual ~NcActionErrorReset();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("error_reset", NcActionErrorReset, ActionType::DEFAULT)
}  // namespace NaviFra
#endif  // NC_ACTION_ERROR_RESET_H