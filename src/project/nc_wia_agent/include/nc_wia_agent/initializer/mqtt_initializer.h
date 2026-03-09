#ifndef NAVIFRA_MQTT_INITIALIZER_H
#define NAVIFRA_MQTT_INITIALIZER_H

#include "core_agent/manager/initializer_manager.h"
#include <ros/ros.h>

namespace NaviFra {

class MQTTInitializer : public Initializer {
public:
    void initialize() override;
    int priority() const override { return 100; }  // config 이후에 실행
private:
    ros::Timer mqtt_check_timer_;
};

REGISTER_INITIALIZER(MQTTInitializer)
}  // namespace NaviFra

#endif  // NAVIFRA_MQTT_INITIALIZER_H
