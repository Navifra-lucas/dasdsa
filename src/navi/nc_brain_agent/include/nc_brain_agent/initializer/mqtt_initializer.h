
#ifndef NAVIFRA_MQTT_INITIALIZER_H
#define NAVIFRA_MQTT_INITIALIZER_H
#pragma once

#include "core_agent/manager/initializer_manager.h"
#include "core_agent/util/config.h"

namespace NaviFra {

class MqttInitializer : public Initializer {
public:
    void initialize();
    int priority() const override { return 100; }
};

// Initializer 자동 등록
REGISTER_INITIALIZER(MqttInitializer)

}  // namespace NaviFra

#endif  // NAVIFRA_MQTT_INITIALIZER_H