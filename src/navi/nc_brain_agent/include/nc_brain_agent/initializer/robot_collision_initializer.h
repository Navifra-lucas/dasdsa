#ifndef NAVIFRA_ROBOT_COLLISION_INITIALIZER_H
#define NAVIFRA_ROBOT_COLLISION_INITIALIZER_H
#pragma once

#include "core_agent/manager/initializer_manager.h"

namespace NaviFra {

class RobotCollisionInitializer : public Initializer {
public:
    void initialize() override;
    int priority() const override { return 60; }  // AlarmManager 이후 실행
};

REGISTER_INITIALIZER(RobotCollisionInitializer)

}  // namespace NaviFra

#endif  // NAVIFRA_ROBOT_COLLISION_INITIALIZER_H
