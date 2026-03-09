#ifndef NC_ACTION_CALIBRATION_STEER_WRITE_H
#define NC_ACTION_CALIBRATION_STEER_WRITE_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionCalibrationSteerWrite : public ActionBase {
public:
    NcActionCalibrationSteerWrite();
    virtual ~NcActionCalibrationSteerWrite();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("calibration_steer_write", NcActionCalibrationSteerWrite, ActionType::DEFAULT)
}  // namespace NaviFra
#endif