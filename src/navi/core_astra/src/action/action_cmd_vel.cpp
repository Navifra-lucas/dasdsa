#include "core_astra/action/action_cmd_vel.h"

#include "core_agent/util/config.h"
#include "core_astra/util/ros_utils.hpp"
#include "core_astra/util/sim_util.hpp"
#include "util/logger.hpp"

#include <geometry_msgs/Twist.h>
#include <ros/node_handle.h>

namespace NaviFra {

ActionCMDVel::ActionCMDVel()
{
    ros::NodeHandle nh_;  // 수명 유지
    cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
}

ActionCMDVel::~ActionCMDVel()
{
    LOG_INFO("ActionCMDVel destroyed");
}

void ActionCMDVel::handle(ActionContext& ctx, Poco::JSON::Object::Ptr req)
{
    // 취소 체크
    if (ctx.cancelled && *ctx.cancelled) {
        ctx.error(name(), ActionError::Cancelled, "Request cancelled");
        return;
    }

    if (simulate_delay_or_drop(ctx, name().c_str()))
        return;

    // auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
    // if (robotInfo->getStatus() == "running") {
    //    LOG_ERROR("robot status is running. can not action on %s", action.c_str());
    //    sendResponseSuccess(source, obj->get("uuid").convert<std::string>(), "fail");
    //    // sendResponseSuccess(source , obj->get("uuid").convert<std::string>(), "fail", "The robot cannot perform the task in running
    //    // status.");
    //    return;
    //}

    try {
        float linearX = req->has("linear") ? req->getValue<float>("linear") : 0.0;
        float linearY = req->has("linear_y") ? req->getValue<float>("linear_y") : 0.0;
        float angulaZ = req->has("angular") ? req->getValue<float>("angular") * 3.14 / 180 : 0.0;

        if (linearX == 0.0 && linearY == 0.0 && angulaZ == 0.0) {
            ctx.ok(name());
            return;
        }
        geometry_msgs::Twist controlMsg;
        controlMsg.linear.x = linearX;
        controlMsg.linear.y = linearY;
        controlMsg.angular.z = angulaZ;
        cmd_vel_.publish(controlMsg);
    }
    catch (Poco::Exception& ex) {
        LOG_ERROR("what %s", ex.displayText().c_str());
    }

    // 성공 응답
    ctx.ok(name());
}

}  // namespace NaviFra
