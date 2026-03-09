#ifndef NAVIFRA_ACTION_MOVE_H
#define NAVIFRA_ACTION_MOVE_H

#include "core_astra/action/action_base.h"

namespace NaviFra {
/**
 * Action: robot_info
 * - 요청 시 로봇 ID를 JSON으로 반환
 * - 응답은 ActionContext.respond()를 통해 비동기로 처리
 */
class ActionMove final : public ActionBase {
public:
    // 실행 메서드 (필수)
    void handle(ActionContext& ctx, Poco::JSON::Object::Ptr req) override;

    // 액션 이름
    std::string name() const override { return "move"; }
};

// ActionRegistry에 등록
REGISTER_ACTION("move", ActionMove, ActionType::DEFAULT)
}  // namespace NaviFra
#endif