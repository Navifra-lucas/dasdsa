#ifndef NAVIFRA_ACTION_GET_ROS_NODES_H
#define NAVIFRA_ACTION_GET_ROS_NODES_H

#include "core_astra/action/action_base.h"

namespace NaviFra {

/**
 * Action: get_ros_nodes
 * - 요청 시 현재 로봇의 모드 ROS Node를 리턴함
 * - 응답은 ActionContext.respond()를 통해 비동기로 처리
 */
class ActionGetROSNodes final : public ActionBase {
public:
    // 실행 메서드 (필수)
    void handle(ActionContext& ctx, Poco::JSON::Object::Ptr req) override;

    // 액션 이름
    std::string name() const override { return "get_ros_nodes"; }
};

// ActionRegistry에 등록
REGISTER_ACTION("get_ros_nodes", ActionGetROSNodes, ActionType::DEFAULT)

}  // namespace NaviFra

#endif  // NAVIFRA_ACTION_GET_ROS_NODES_H