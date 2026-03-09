#ifndef NC_ACTION_CONTRO_EXD_H
#define NC_ACTION_CONTRO_EXD_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionControEXD : public ActionBase {
public:
    NcActionControEXD();
    virtual ~NcActionControEXD();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("plc", NcActionControEXD, ActionType::VOLVO)
}  // namespace NaviFra
#endif