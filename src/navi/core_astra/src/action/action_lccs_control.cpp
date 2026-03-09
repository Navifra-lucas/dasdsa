#include "core_astra/action/action_lccs_control.h"

#include "core_agent/util/config.h"
#include "core_astra/util/ros_utils.hpp"
#include "core_astra/util/sim_util.hpp"
#include "util/logger.hpp"

#include <std_msgs/Bool.h>
#include <ros/node_handle.h>

namespace NaviFra {

ActionLCCSControl::ActionLCCSControl()
{
    ros::NodeHandle nh_;  // 수명 유지
    lccs_pub_ = nh_.advertise<std_msgs::Bool>("/navifra/lccs_use", 10);
}

ActionLCCSControl::~ActionLCCSControl()
{
    LOG_INFO("ActionLCCSControl destroyed");
}

void ActionLCCSControl::handle(ActionContext& ctx, Poco::JSON::Object::Ptr req)
{
    // 취소 체크
    if (ctx.cancelled && *ctx.cancelled) {
        ctx.error(name(), ActionError::Cancelled, "Request cancelled");
        return;
    }

    if (simulate_delay_or_drop(ctx, name().c_str()))
        return;

    try {
        Poco::JSON::Object::Ptr dataObj = req->getObject("data");
        bool OnOff = dataObj->has("OnOff") ? dataObj->getValue<bool>("OnOff") : false;

        std_msgs::Bool str_msg;
        str_msg.data = OnOff;
        lccs_pub_.publish(str_msg);
    }
    catch (Poco::Exception& ex) {
        LOG_ERROR("what %s", ex.displayText().c_str());
    }

    // 성공 응답
    ctx.ok(name());

    LOG_INFO("ActionLCCSControl complete");
}

}  // namespace NaviFra
