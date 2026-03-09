#ifndef NC_ACTION_AMR_BASIC_H
#define NC_ACTION_AMR_BASIC_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionAMRBasic : public ActionBase {
public:
    NcActionAMRBasic();
    virtual ~NcActionAMRBasic();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("basic", NcActionAMRBasic, ActionType::DEFAULT)
}  // namespace NaviFra
#endif  // NC_ACTION_AMR_BASIC_H