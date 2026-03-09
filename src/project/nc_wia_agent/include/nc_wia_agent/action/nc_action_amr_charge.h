#ifndef NC_ACTION_AMR_MODE_H
#define NC_ACTION_AMR_MODE_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionAmrCharge : public ActionBase {
public:
    NcActionAmrCharge();
    virtual ~NcActionAmrCharge();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("mode", NcActionAmrCharge, ActionType::DEFAULT)
}  // namespace NaviFra
#endif  // NC_ACTION_AMR_MODE_H