#ifndef NC_ACTION_UPDATE_PRESETS_H
#define NC_ACTION_UPDATE_PRESETS_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionUpdatePresets : public ActionBase {
public:
    NcActionUpdatePresets();
    virtual ~NcActionUpdatePresets();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("update_presets", NcActionUpdatePresets, ActionType::VOLVO)

}  // namespace NaviFra

#endif  // !NC_ACTION_UPDATE_PRESETS_H
