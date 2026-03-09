#ifndef NAVIFRA_MAP_SYNC_INITIALIZER_H
#define NAVIFRA_MAP_SYNC_INITIALIZER_H
#pragma once

#include "core_agent/manager/initializer_manager.h"

namespace NaviFra {

class MapSyncInitializer : public Initializer {
public:
    void initialize() override;
    int priority() const override { return 30; }  // RobotVerify 이후 실행되도록 우선순위 설정
    std::string s_path_ = "";
};

REGISTER_INITIALIZER(MapSyncInitializer)

}  // namespace NaviFra

#endif  // NAVIFRA_MAP_SYNC_INITIALIZER_H
