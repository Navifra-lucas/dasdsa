#ifndef NC_ACTION_CALIBRATION_VOLVO_H
#define NC_ACTION_CALIBRATION_VOLVO_H

#include <nc_brain_agent/action/nc_action_calibration.h>

namespace NaviFra {
class NcActionCalibrationVolvo : public NcActionCalibration {
public:
    NcActionCalibrationVolvo();
    virtual ~NcActionCalibrationVolvo();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("calibration", NcActionCalibrationVolvo, ActionType::VOLVO)
}  // namespace NaviFra
#endif