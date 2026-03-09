#include "core_astra/action/action_move.h"

#include "core_astra/util/sim_util.hpp"
#include "core_msgs/CommonString.h"
#include "util/logger.hpp"

#include <Poco/JSON/Array.h>
#include <Poco/JSON/Object.h>
#include <Poco/UUIDGenerator.h>
#include <ros/ros.h>

#include <sstream>
#include <stdexcept>

namespace NaviFra {

void ActionMove::handle(ActionContext& ctx, Poco::JSON::Object::Ptr req)
{
    LOG_INFO("[ActionMove] 요청 처리 시작");

    // 1) 취소 체크
    if (ctx.cancelled && *ctx.cancelled) {
        ctx.error(name(), ActionError::Cancelled, "Request cancelled");
        return;
    }

    // 2) 시뮬레이터용 드랍/딜레이 훅
    if (simulate_delay_or_drop(ctx, name().c_str()))
        return;

    // 3) 입력 파라미터 검증
    if (!req || !req->has("node_id")) {
        ctx.error(name(), ActionError::InvalidArgument, "missing 'node_id'");
        LOG_ERROR("[ActionMove] missing 'node_id' in request");
        return;
    }
    const std::string nodeId = req->get("node_id").convert<std::string>();
    if (nodeId.empty()) {
        ctx.error(name(), ActionError::InvalidArgument, "'node_id' is empty");
        LOG_ERROR("[ActionMove] 'node_id' is empty");
        return;
    }

    // 4) JSON 구성 (Poco JSON만 사용)
    Poco::JSON::Object::Ptr root = new Poco::JSON::Object;
    root->set("action", "add_task");

    Poco::JSON::Array::Ptr data = new Poco::JSON::Array;
    Poco::JSON::Object::Ptr item = new Poco::JSON::Object;
    item->set("type", "move");
    item->set("uuid", Poco::UUIDGenerator::defaultGenerator().createRandom().toString());  // task uuid
    item->set("target_node", nodeId);
    data->add(item);

    root->set("data", data);
    root->set("uuid", Poco::UUIDGenerator::defaultGenerator().createRandom().toString());  // request uuid

    std::ostringstream oss;
    root->stringify(oss);  // 보기 좋게: root->stringify(oss, 2)

    // 5) 서비스 클라이언트 생성 및 호출
    //    CommonString.srv 가 아래와 같은 구조라고 가정:
    //    --- Request ---
    //    string data
    //    --- Response ---
    //    string data
    ros::NodeHandle nh;  // 수명 유지
    ros::ServiceClient client = nh.serviceClient<core_msgs::CommonString>("/nc_task_manager_srv/task_add");

    core_msgs::CommonString srv;
    srv.request.data = oss.str();

    if (!client.call(srv)) {
        // 호출 실패
        ctx.error(name(), ActionError::Internal, "service call failed");
        LOG_ERROR("[ActionMove] service call failed: /nc_task_manager_srv/task_add");
        return;
    }

    // 호출 성공
    NLOG(info) << "[ActionMove] move request success, resp=" << srv.response.reason;

    // 6) 액션 성공 처리 (필요하면 payload를 넘겨도 됨)
    ctx.ok(name());

    LOG_INFO("[ActionMove] 응답 완료");
}

}  // namespace NaviFra
