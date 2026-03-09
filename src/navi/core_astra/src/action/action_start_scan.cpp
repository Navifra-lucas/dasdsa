#include "core_astra/action/action_start_scan.h"

#include "core_agent/util/config.h"
#include "core_astra/util/sim_util.hpp"
#include "util/logger.hpp"

#include <geometry_msgs/Twist.h>
#include <ros/node_handle.h>

namespace NaviFra {

ActionStartSCAN::ActionStartSCAN()
{
    // ros::NodeHandle nh_;  // 수명 유지
    // cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
}

ActionStartSCAN::~ActionStartSCAN()
{
    LOG_INFO("ActionCMDVel destroyed");
}

void ActionStartSCAN::handle(ActionContext& ctx, Poco::JSON::Object::Ptr req)
{
    // 취소 체크
    if (ctx.cancelled && *ctx.cancelled) {
        ctx.error(name(), ActionError::Cancelled, "Request cancelled");
        return;
    }

    if (simulate_delay_or_drop(ctx, name().c_str()))
        return;

    // 성공 응답
    ctx.ok(name());
}

}  // namespace NaviFra
