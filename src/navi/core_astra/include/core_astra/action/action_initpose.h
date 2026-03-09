#ifndef NAVIFRA_ACTION_INIT_POSE_H
#define NAVIFRA_ACTION_INIT_POSE_H

#include "core_astra/action/action_base.h"

#include <ros/publisher.h>

namespace NaviFra {
/**
 * Action: init_pose
 * - 요청 시 현재 로봇의 모드 Pose를 변경 함
 * - 응답은 ActionContext.respond()를 통해 비동기로 처리
 */
class ActionInitPose final : public ActionBase {
public:
    ActionInitPose();

private:
    // 실행 메서드 (필수)
    void handle(ActionContext& ctx, Poco::JSON::Object::Ptr req) override;

    // 액션 이름
    std::string name() const override { return "init_pose"; }
    ros::Publisher initpose_pub_;
    ros::Publisher initpose_correct_position_pub_;
};

// ActionRegistry에 등록
REGISTER_ACTION("init_pose", ActionInitPose, ActionType::DEFAULT)
}  // namespace NaviFra

#endif  //