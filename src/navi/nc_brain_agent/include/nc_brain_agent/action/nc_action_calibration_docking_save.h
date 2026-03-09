#ifndef NC_ACTION_CALIBRATION_DOCKING_SAVE_H
#define NC_ACTION_CALIBRATION_DOCKING_SAVE_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionCalibrationDockingSave : public ActionBase {
public:
    NcActionCalibrationDockingSave();
    virtual ~NcActionCalibrationDockingSave();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("calibration_docking_save", NcActionCalibrationDockingSave, ActionType::DEFAULT)
}  // namespace NaviFra
#endif