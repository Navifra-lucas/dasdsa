#ifndef NC_ACTION_AMR_MANUAL_H
#define NC_ACTION_AMR_MANUAL_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionAMRManual : public ActionBase {
public:
    NcActionAMRManual();
    virtual ~NcActionAMRManual();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("manual", NcActionAMRManual, ActionType::DEFAULT)
}  // namespace NaviFra
#endif  // NC_ACTION_AMR_MANUAL_H