#ifndef NC_ACTION_TEACHED_H
#define NC_ACTION_TEACHED_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionTeached : public ActionBase {
public:
    NcActionTeached();
    virtual ~NcActionTeached();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("teached", NcActionTeached, ActionType::DEFAULT)
}  // namespace NaviFra
#endif