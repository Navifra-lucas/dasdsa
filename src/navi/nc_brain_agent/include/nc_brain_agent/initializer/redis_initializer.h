#ifndef NAVIFRA_REDIS_INITIALIZER_H
#define NAVIFRA_REDIS_INITIALIZER_H
#pragma once

#include "core_agent/manager/initializer_manager.h"

namespace NaviFra {

class RedisInitializer : public Initializer {
public:
    void initialize();
    int priority() const override { return 100; }
};
// Initializer 자동 등록
REGISTER_INITIALIZER(RedisInitializer)

}  // namespace NaviFra
#endif  // NAVIFRA_REDIS_INITIALIZER_H