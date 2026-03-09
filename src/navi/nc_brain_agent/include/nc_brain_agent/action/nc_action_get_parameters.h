#ifndef NC_ACTION_GET_PARAMETERS_H
#define NC_ACTION_GET_PARAMETERS_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionGetParameters : public ActionBase {
public:
    NcActionGetParameters();
    virtual ~NcActionGetParameters();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("get_parameters", NcActionGetParameters, ActionType::DEFAULT)
}  // namespace NaviFra
#endif