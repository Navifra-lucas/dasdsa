#ifndef NAVIFRA_ACTION_MANAGER_INITIALIZER_H
#define NAVIFRA_ACTION_MANAGER_INITIALIZER_H
#pragma once

#include "core_agent/manager/initializer_manager.h"

namespace NaviFra {

class ActionManagerInitializer : public Initializer {
public:
    void initialize() override;
    int priority() const override { return 70; }  // RobotCollision 이후 실행
};

REGISTER_INITIALIZER(ActionManagerInitializer)

}  // namespace NaviFra

#endif  // NAVIFRA_ACTION_MANAGER_INITIALIZER_H
