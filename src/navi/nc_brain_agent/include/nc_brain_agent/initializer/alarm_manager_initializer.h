#ifndef NAVIFRA_ALARM_MANAGER_INITIALIZER_H
#define NAVIFRA_ALARM_MANAGER_INITIALIZER_H
#pragma once

#include "core_agent/manager/initializer_manager.h"

namespace NaviFra {

class AlarmManagerInitializer : public Initializer {
public:
    void initialize() override;
    int priority() const override { return 50; }  // RobotGroup 이후 실행
};

REGISTER_INITIALIZER(AlarmManagerInitializer)

}  // namespace NaviFra

#endif  // NAVIFRA_ALARM_MANAGER_INITIALIZER_H
