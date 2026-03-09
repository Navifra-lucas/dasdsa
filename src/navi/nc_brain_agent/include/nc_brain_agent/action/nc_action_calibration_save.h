#ifndef NC_ACTION_CALIBRATION_SAVE_H
#define NC_ACTION_CALIBRATION_SAVE_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionCalibrationSave : public ActionBase {
public:
    NcActionCalibrationSave();
    virtual ~NcActionCalibrationSave();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("calibration_save", NcActionCalibrationSave, ActionType::DEFAULT)
}  // namespace NaviFra
#endif