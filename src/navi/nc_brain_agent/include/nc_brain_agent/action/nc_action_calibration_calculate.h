#ifndef NC_ACTION_CALIBRATION_CALCULATE_H
#define NC_ACTION_CALIBRATION_CALCULATE_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionCalibrationCalculate : public ActionBase {
public:
    NcActionCalibrationCalculate();
    virtual ~NcActionCalibrationCalculate();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("calibration_calculate", NcActionCalibrationCalculate, ActionType::DEFAULT)
}  // namespace NaviFra
#endif