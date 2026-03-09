#include "core_astra/action/action_robot_info.h"

#include "core_agent/util/config.h"
#include "util/logger.hpp"

namespace NaviFra {

void ActionRobotInfo::handle(ActionContext& ctx, Poco::JSON::Object::Ptr /*req*/)
{
    LOG_INFO("[ActionRobotInfo] 요청 처리 시작");

    // 취소 체크
    if (ctx.cancelled && *ctx.cancelled) {
        ctx.error(name(), ActionError::Cancelled, "Request cancelled");
        return;
    }

    // 결과 JSON 구성
    Poco::JSON::Object::Ptr data = new Poco::JSON::Object;
    data->set("id", Config::instance().getString("robot_id", "unknown"));

    // 성공 응답
    ctx.ok(name(), data);

    LOG_INFO("[ActionRobotInfo] 응답 완료");
}

}  // namespace NaviFra
