#ifndef NC_ACTION_CALIBRATION_RESET_H
#define NC_ACTION_CALIBRATION_RESET_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionCalibrationReset : public ActionBase {
public:
    NcActionCalibrationReset();
    virtual ~NcActionCalibrationReset();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("calibration_reset", NcActionCalibrationReset, ActionType::DEFAULT)
}  // namespace NaviFra
#endif