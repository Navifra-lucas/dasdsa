#ifndef NAVIFRA_MAP_INITIALIZER_H
#define NAVIFRA_MAP_INITIALIZER_H

#include "core_agent/manager/initializer_manager.h"

#include <ros/ros.h>

namespace NaviFra {

class MapInitializer : public Initializer {
public:
    void initialize() override;
    int priority() const override { return 90; }  // mqtt 이전에 실행

private:
    ros::Publisher map_pub_;
};

REGISTER_INITIALIZER(MapInitializer)
}  // namespace NaviFra

#endif  // NAVIFRA_MAP_INITIALIZER_H
