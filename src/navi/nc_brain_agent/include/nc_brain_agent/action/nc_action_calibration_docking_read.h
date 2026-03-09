#ifndef NC_ACTION_CALIBRATION_DOCKING_READ_H
#define NC_ACTION_CALIBRATION_DOCKING_READ_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionCalibrationDockingRead : public ActionBase {
public:
    NcActionCalibrationDockingRead();
    virtual ~NcActionCalibrationDockingRead();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("calibration_docking_read", NcActionCalibrationDockingRead, ActionType::DEFAULT)
}  // namespace NaviFra
#endif