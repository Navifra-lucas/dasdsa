#ifndef NC_ACTION_AVOID_PERMISSION_H
#define NC_ACTION_AVOID_PERMISSION_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionAvoidPerMission : public ActionBase {
public:
    NcActionAvoidPerMission();
    virtual ~NcActionAvoidPerMission();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("barcode_reader", NcActionAvoidPerMission, ActionType::DEFAULT)
}  // namespace NaviFra
#endif