#ifndef NAVIFRA_MOTOR_INITIALIZER_H
#define NAVIFRA_MOTOR_INITIALIZER_H

#include "core_agent/manager/initializer_manager.h"

namespace NaviFra {

class MotorInitializer : public Initializer {
public:
    void initialize() override;
    int priority() const override { return 120; }  // Config/MQTT 이후
};

REGISTER_INITIALIZER(MotorInitializer)

}  // namespace NaviFra

#endif  // NAVIFRA_MOTOR_INITIALIZER_H
