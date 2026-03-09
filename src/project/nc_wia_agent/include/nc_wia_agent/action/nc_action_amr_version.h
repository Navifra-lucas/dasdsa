#ifndef NC_ACTION_AMR_VERSION_H
#define NC_ACTION_AMR_VERSION_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionAMRVersion : public ActionBase {
public:
    NcActionAMRVersion();
    virtual ~NcActionAMRVersion();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("amr_ver", NcActionAMRVersion, ActionType::DEFAULT)
}  // namespace NaviFra
#endif  // NC_ACTION_AMR_VERSION_H