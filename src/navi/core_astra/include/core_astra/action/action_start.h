#ifndef NAVIFRA_ACTION_START_H
#define NAVIFRA_ACTION_START_H

#include "core_astra/action/action_base.h"

#include <ros/publisher.h>

#include <atomic>

namespace NaviFra {
/**
 * Action: ActionStart
 * - 요청 시 LCCS OnOff 상태를 제어 함
 * - 응답은 ActionContext.respond()를 통해 비동기로 처리
 */
class ActionStart final : public ActionBase {
public:
    ActionStart();
    ~ActionStart();

private:
    // 실행 메서드 (필수)
    void handle(ActionContext& ctx, Poco::JSON::Object::Ptr req) override;

    // 액션 이름
    std::string name() const override { return "acs_start"; }
    ros::Publisher cmd_pub_;
};

// ActionRegistry에 등록
REGISTER_ACTION("acs_start", ActionStart, ActionType::DEFAULT)
}  // namespace NaviFra

#endif  // NAVIFRA_ACTION_LCCS_CONTROL_H