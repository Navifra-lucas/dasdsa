#ifndef NAVIFRA_ACTION_LCCS_CONTROL_H
#define NAVIFRA_ACTION_LCCS_CONTROL_H

#include "core_astra/action/action_base.h"

#include <ros/publisher.h>

#include <atomic>

namespace NaviFra {
/**
 * Action: ActionLCCSControl
 * - 요청 시 LCCS OnOff 상태를 제어 함
 * - 응답은 ActionContext.respond()를 통해 비동기로 처리
 */
class ActionLCCSControl final : public ActionBase {
public:
    ActionLCCSControl();
    ~ActionLCCSControl();

private:
    // 실행 메서드 (필수)
    void handle(ActionContext& ctx, Poco::JSON::Object::Ptr req) override;

    // 액션 이름
    std::string name() const override { return "control_lccs"; }
    ros::Publisher lccs_pub_;
};

// ActionRegistry에 등록
REGISTER_ACTION("control_lccs", ActionLCCSControl, ActionType::DEFAULT)
}  // namespace NaviFra

#endif  // NAVIFRA_ACTION_LCCS_CONTROL_H