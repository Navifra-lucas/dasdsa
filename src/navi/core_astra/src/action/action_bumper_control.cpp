#include "core_astra/action/action_bumper_control.h"

#include "core_agent/util/config.h"
#include "core_astra/util/ros_utils.hpp"
#include "core_astra/util/sim_util.hpp"
#include "util/logger.hpp"

#include <std_msgs/String.h>
#include <ros/node_handle.h>

namespace NaviFra {

ActionBumperControl::ActionBumperControl()
{
    ros::NodeHandle nh_;  // 수명 유지
    pub_bumper_ = nh_.advertise<std_msgs::String>("output_command", 1, false);
}

ActionBumperControl::~ActionBumperControl()
{
    LOG_INFO("ActionBumperControl destroyed");
}

void ActionBumperControl::handle(ActionContext& ctx, Poco::JSON::Object::Ptr req)
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

        std_msgs::String str_msg;
        if(OnOff) {
            str_msg.data = "bumper_bypass_on";
        }
        else {
            str_msg.data = "bumper_bypass_off";
        }
        pub_bumper_.publish(str_msg);
    }
    catch (Poco::Exception& ex) {
        LOG_ERROR("what %s", ex.displayText().c_str());
    }

    // 성공 응답
    ctx.ok(name());

    LOG_INFO("ActionBumperControl complete");
}

}  // namespace NaviFra
