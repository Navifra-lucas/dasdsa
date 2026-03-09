#ifndef NAVIFRA_PUBLISHER_INITIALIZER_H
#define NAVIFRA_PUBLISHER_INITIALIZER_H

#include "core_agent/manager/initializer_manager.h"

namespace NaviFra {

class PublisherInitializer : public Initializer {
public:
    void initialize() override;
    int priority() const override { return 150; }  // MQTT 이후
};

REGISTER_INITIALIZER(PublisherInitializer)

}  // namespace NaviFra

#endif  // NAVIFRA_PUBLISHER_INITIALIZER_H
