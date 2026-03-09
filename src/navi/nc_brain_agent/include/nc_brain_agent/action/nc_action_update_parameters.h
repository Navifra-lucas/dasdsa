#ifndef NC_ACTION_UPDATE_PARAMETERS_H
#define NC_ACTION_UPDATE_PARAMETERS_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionUpdateParameters : public ActionBase {
public:
    NcActionUpdateParameters();
    virtual ~NcActionUpdateParameters();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("update_parameters", NcActionUpdateParameters, ActionType::DEFAULT)
}  // namespace NaviFra
#endif