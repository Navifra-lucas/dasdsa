#ifndef NAVIFRA_ROBOT_VERIFY_INITIALIZER_H
#define NAVIFRA_ROBOT_VERIFY_INITIALIZER_H
#pragma once

#include "core_agent/manager/initializer_manager.h"

namespace NaviFra {

class RobotVerifyInitializer : public Initializer {
public:
    void initialize() override;
    int priority() const override { return 20; }  // Redis 이후 실행되도록 우선순위 부여
};

REGISTER_INITIALIZER(RobotVerifyInitializer)

}  // namespace NaviFra

#endif  // NAVIFRA_ROBOT_VERIFY_INITIALIZER_H
