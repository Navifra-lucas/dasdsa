#ifndef NC_ACTION_CALIBRATION_H
#define NC_ACTION_CALIBRATION_H

#include <core_agent/action/action_base.h>

namespace NaviFra {
class NcActionCalibration : public ActionBase {
public:
    NcActionCalibration();
    virtual ~NcActionCalibration();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("calibration", NcActionCalibration, ActionType::DEFAULT)
}  // namespace NaviFra
#endif