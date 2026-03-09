#include "core_astra/action/action_lifter_control.h"

#include "core_agent/util/config.h"
#include "core_astra/util/ros_utils.hpp"
#include "core_astra/util/sim_util.hpp"
#include "util/logger.hpp"

#include <std_msgs/String.h>
#include <ros/node_handle.h>
#include <iostream>
#include <sstream>

namespace NaviFra {

ActionLifterControl::ActionLifterControl()
{
    ros::NodeHandle nh_;  // 수명 유지
    pub_lift_ = nh_.advertise<std_msgs::String>("output_command", 1, false);
}

ActionLifterControl::~ActionLifterControl()
{
    LOG_INFO("ActionLifterControl destroyed");
}

void ActionLifterControl::handle(ActionContext& ctx, Poco::JSON::Object::Ptr req)
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

        std::ostringstream oss;
        dataObj->stringify(oss);
        NLOG(info)<<"HMI Lifter Cmd : "<<oss.str();
        std_msgs::String str_msg;
        if(OnOff) {
            str_msg.data = "pin_up";
        }
        else {
            str_msg.data = "pin_down";
        }
        pub_lift_.publish(str_msg);
    }
    catch (Poco::Exception& ex) {
        LOG_ERROR("what %s", ex.displayText().c_str());
    }

    // 성공 응답
    ctx.ok(name());

    LOG_INFO("ActionLifterControl complete");
}

}  // namespace NaviFra
