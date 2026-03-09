#ifndef NC_ACTION_CALIBRATION_STEER_ZERO_SET_H
#define NC_ACTION_CALIBRATION_STEER_ZERO_SET_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionCalibrationSteerZeroSet : public ActionBase {
public:
    NcActionCalibrationSteerZeroSet();
    virtual ~NcActionCalibrationSteerZeroSet();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("calibration_steer_zeroset", NcActionCalibrationSteerZeroSet, ActionType::DEFAULT)
}  // namespace NaviFra
#endif