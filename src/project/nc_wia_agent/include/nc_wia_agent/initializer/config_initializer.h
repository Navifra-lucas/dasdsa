#ifndef NAVIFRA_CONFIG_INITIALIZER_H
#define NAVIFRA_CONFIG_INITIALIZER_H

#include "core_agent/manager/initializer_manager.h"

namespace NaviFra {

class ConfigInitializer : public Initializer {
public:
    void initialize() override;
    int priority() const override { return 10; }  // 가장 먼저 실행
};

REGISTER_INITIALIZER(ConfigInitializer)
}  // namespace NaviFra

#endif  // NAVIFRA_CONFIG_INITIALIZER_H
