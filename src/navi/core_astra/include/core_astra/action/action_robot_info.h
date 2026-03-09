#ifndef NAVIFRA_ACTION_ROBOT_INFO_H
#define NAVIFRA_ACTION_ROBOT_INFO_H

#include "core_astra/action/action_base.h"

namespace NaviFra {

/**
 * Action: robot_info
 * - 요청 시 로봇 ID를 JSON으로 반환
 * - 응답은 ActionContext.respond()를 통해 비동기로 처리
 */
class ActionRobotInfo final : public ActionBase {
public:
    // 실행 메서드 (필수)
    void handle(ActionContext& ctx, Poco::JSON::Object::Ptr req) override;

    // 액션 이름
    std::string name() const override { return "robot_info"; }
};

// ActionRegistry에 등록
REGISTER_ACTION("robot_info", ActionRobotInfo, ActionType::DEFAULT)

}  // namespace NaviFra

#endif  // NAVIFRA_ACTION_ROBOT_INFO_H