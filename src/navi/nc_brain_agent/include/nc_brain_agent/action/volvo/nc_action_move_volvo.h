#ifndef NC_ACTION_MOVE_VOLVO_H
#define NC_ACTION_MOVE_VOLVO_H

#include <nc_brain_agent/action/nc_action_move.h>

namespace NaviFra {
class NcActionMoveVolvo : public NcActionMove {
public:
    NcActionMoveVolvo();
    virtual ~NcActionMoveVolvo();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("move", NcActionMoveVolvo, ActionType::VOLVO)
}  // namespace NaviFra
#endif