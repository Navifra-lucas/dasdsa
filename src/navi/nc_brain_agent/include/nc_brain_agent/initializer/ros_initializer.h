#ifndef NAVIFRA_ROS_INITIALIZER_H
#define NAVIFRA_ROS_INITIALIZER_H
#pragma once

#include "core_agent/manager/initializer_manager.h"

namespace NaviFra {

class ROSInitializer : public Initializer {
public:
    void initialize() override;
    int priority() const override { return 80; }  // ActionManager 이후 실행
};

REGISTER_INITIALIZER(ROSInitializer)

}  // namespace NaviFra

#endif  // NAVIFRA_ROS_INITIALIZER_H
