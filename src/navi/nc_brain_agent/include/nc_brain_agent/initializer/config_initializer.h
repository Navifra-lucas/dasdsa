#ifndef NAVIFRA_CONFIG_INITIALIZER_H
#define NAVIFRA_CONFIG_INITIALIZER_H

#pragma once

#include "core_agent/manager/initializer_manager.h"

namespace NaviFra {

class ConfigInitializer : public Initializer {
public:
    void initialize() override;
    int priority() const override { return 1; }  // 가장 먼저 실행
};

REGISTER_INITIALIZER(ConfigInitializer)
}  // namespace NaviFra

#endif  // NAVIFRA_CONFIG_INITIALIZER_H