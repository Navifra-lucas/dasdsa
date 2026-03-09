#ifndef NAVIFRA_ROBOT_GROUP_INITIALIZER_H
#define NAVIFRA_ROBOT_GROUP_INITIALIZER_H
#pragma once

#include "core_agent/manager/initializer_manager.h"

namespace NaviFra {

class RobotGroupInitializer : public Initializer {
public:
    void initialize() override;
    int priority() const override { return 40; }  // MapSync 이후 실행되도록 설정
};

REGISTER_INITIALIZER(RobotGroupInitializer)

}  // namespace NaviFra

#endif  // NAVIFRA_ROBOT_GROUP_INITIALIZER_H
