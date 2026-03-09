#include "core_astra/action/action_start.h"

#include "core_agent/util/config.h"
#include "core_astra/util/ros_utils.hpp"
#include "core_astra/util/sim_util.hpp"
#include "util/logger.hpp"

#include <std_msgs/String.h>
#include <ros/node_handle.h>

namespace NaviFra {

ActionStart::ActionStart()
{
    ros::NodeHandle nh_;  // 수명 유지
    cmd_pub_ = nh_.advertise<std_msgs::String>("/navifra/cmd", 10);
}

ActionStart::~ActionStart()
{
    LOG_INFO("ActionStart destroyed");
}

void ActionStart::handle(ActionContext& ctx, Poco::JSON::Object::Ptr req)
{
    // 취소 체크
    if (ctx.cancelled && *ctx.cancelled) {
        ctx.error(name(), ActionError::Cancelled, "Request cancelled");
        return;
    }

    if (simulate_delay_or_drop(ctx, name().c_str()))
        return;

    NLOG(info)<<"HMI Start Cmd";
    std_msgs::String str_msg;
    str_msg.data = "resume";
    cmd_pub_.publish(str_msg);

    // 성공 응답
    ctx.ok(name());

    LOG_INFO("ActionStart complete");
}

}  // namespace NaviFra
