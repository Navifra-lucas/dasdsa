#ifndef NAVIFRA_CONFIG_INITIALIZER_H
#define NAVIFRA_CONFIG_INITIALIZER_H
#pragma once
#include "core_agent/manager/initializer_manager.h"

namespace NaviFra {
class ConfigInitializer : public Initializer {
public:
    virtual ~ConfigInitializer() {}
    void initialize() override;
    int priority() const override { return 0; }  // RobotVerify 이후 실행되도록 우선순위 설정
};

REGISTER_INITIALIZER(ConfigInitializer)
}  // namespace NaviFra

#endif  // NAVIFRA_CONFIG_INITIALIZER_H