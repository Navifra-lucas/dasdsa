#ifndef NC_ACTION_AMR_INIT_H
#define NC_ACTION_AMR_INIT_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionAMRInit : public ActionBase {
public:
    NcActionAMRInit();
    virtual ~NcActionAMRInit();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("amr_init", NcActionAMRInit, ActionType::DEFAULT)
}  // namespace NaviFra
#endif  // NC_ACTION_AMR_INIT_H