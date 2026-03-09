#ifndef NAVIFRA_ROBOT_COLLISION_INITIALIZER_H
#define NAVIFRA_ROBOT_COLLISION_INITIALIZER_H
#pragma once
#include "core_agent/manager/initializer_manager.h"

namespace NaviFra {
class RobotCollisionInitializer : public Initializer {
public:
    virtual ~RobotCollisionInitializer() {}
    void initialize() override;
    int priority() const override { return 17; }  // RobotVerify 이후 실행되도록 우선순위 설정
};

REGISTER_INITIALIZER(RobotCollisionInitializer)
}  // namespace NaviFra

#endif  // NAVIFRA_ROBOT_COLLISION_INITIALIZER_H