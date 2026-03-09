#ifndef NC_ACTION_EXD_JIG
#define NC_ACTION_EXD_JIG

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionEXDJIG : public ActionBase {
public:
    NcActionEXDJIG();
    virtual ~NcActionEXDJIG();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("exd_jig", NcActionEXDJIG, ActionType::VOLVO)
}  // namespace NaviFra
#endif