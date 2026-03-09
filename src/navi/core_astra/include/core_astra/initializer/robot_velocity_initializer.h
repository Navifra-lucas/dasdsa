#ifndef NAVIFRA_ROBOT_VELOCITY_INITIALIZER_H
#define NAVIFRA_ROBOT_VELOCITY_INITIALIZER_H
#pragma once
#include "core_agent/manager/initializer_manager.h"

namespace NaviFra {
class RobotVelocityInitializer : public Initializer {
public:
    virtual ~RobotVelocityInitializer() {}
    void initialize() override;
    int priority() const override { return 16; }  // RobotVerify 이후 실행되도록 우선순위 설정
};

REGISTER_INITIALIZER(RobotVelocityInitializer)
}  // namespace NaviFra

#endif  // NAVIFRA_ROBOT_VELOCITY_INITIALIZER_H