#ifndef NAVIFRA_ACTION_CMD_VEL_H
#define NAVIFRA_ACTION_CMD_VEL_H

#include "core_astra/action/action_base.h"

#include <ros/publisher.h>

#include <atomic>

namespace NaviFra {
/**
 * Action: CMDVel
 * - 요청 시 로봇의 선속도 각속도를 변경 함
 * - 응답은 ActionContext.respond()를 통해 비동기로 처리
 */
class ActionCMDVel final : public ActionBase {
public:
    ActionCMDVel();
    ~ActionCMDVel();

private:
    // 실행 메서드 (필수)
    void handle(ActionContext& ctx, Poco::JSON::Object::Ptr req) override;

    // 액션 이름
    std::string name() const override { return "control"; }
    ros::Publisher cmd_vel_;
};

// ActionRegistry에 등록
REGISTER_ACTION("control", ActionCMDVel, ActionType::DEFAULT)
}  // namespace NaviFra

#endif  // NAVIFRA_ACTION_CMD_VEL_H