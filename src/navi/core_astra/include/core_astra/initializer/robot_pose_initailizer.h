#ifndef NAVIFRA_ROBOT_POSE_INITIALIZER_H
#define NAVIFRA_ROBOT_POSE_INITIALIZER_H

#pragma once
#include "core_agent/manager/initializer_manager.h"

namespace NaviFra {

class RobotPoseInitializer : public Initializer {
public:
    RobotPoseInitializer() = default;
    virtual ~RobotPoseInitializer();
    void initialize() override;

    int priority() const override { return 15; }  // RobotVerify 이후 실행되도록 우선순위 설정
};

REGISTER_INITIALIZER(RobotPoseInitializer)

}  // namespace NaviFra

#endif  // NAVIFRA_ROBOT_STATUS_INITIALIZER_H