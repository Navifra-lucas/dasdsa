#ifndef NAVIFRA_BACKEND_HEALTHCHECK_INITIALIZER_H
#define NAVIFRA_BACKEND_HEALTHCHECK_INITIALIZER_H
#pragma once
#include "core_agent/manager/initializer_manager.h"

namespace NaviFra {
class BackendHealthCheckInitializer : public Initializer {
public:
    void initialize();

    int priority() const override { return 2; }  // 가장 먼저 실행
};

REGISTER_INITIALIZER(BackendHealthCheckInitializer)
}  // namespace NaviFra

#endif  // NAVIFRA_BACKEND_HEALTHCHECK_INITIALIZER_H