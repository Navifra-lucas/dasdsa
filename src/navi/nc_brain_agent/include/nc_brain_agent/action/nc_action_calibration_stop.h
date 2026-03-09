#ifndef NC_ACTION_CALIBRATION_STOP_H
#define NC_ACTION_CALIBRATION_STOP_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionCalibrationStop : public ActionBase {
public:
    NcActionCalibrationStop();
    virtual ~NcActionCalibrationStop();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("calibration_stop", NcActionCalibrationStop, ActionType::DEFAULT)
}  // namespace NaviFra
#endif