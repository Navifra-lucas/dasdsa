#ifndef NC_ACTION_CONV_CMD_H
#define NC_ACTION_CONV_CMD_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionConvCMD : public ActionBase {
public:
    NcActionConvCMD();
    virtual ~NcActionConvCMD();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("conv_cmd", NcActionConvCMD, ActionType::DEFAULT)
}  // namespace NaviFra
#endif  // NC_ACTION_CONV_CMD_H