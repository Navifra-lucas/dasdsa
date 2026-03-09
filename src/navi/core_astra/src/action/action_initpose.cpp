#include "core_astra/action/action_initpose.h"

#include "core_agent/util/config.h"
#include "core_astra/util/sim_util.hpp"
#include "util/logger.hpp"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/node_handle.h>
#include <std_msgs/String.h>

namespace NaviFra {

ActionInitPose::ActionInitPose()
{
    ros::NodeHandle nh;
    initpose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
    initpose_correct_position_pub_ = nh.advertise<std_msgs::String>("correction_init_pos", 1);
}

void ActionInitPose::handle(ActionContext& ctx, Poco::JSON::Object::Ptr req)
{
    LOG_INFO("[ActionInitPose] 요청 처리 시작");

    // 취소 체크
    if (ctx.cancelled && *ctx.cancelled) {
        ctx.error(name(), ActionError::Cancelled, "Request cancelled");
        return;
    }

    if (simulate_delay_or_drop(ctx, name().c_str()))
        return;

    geometry_msgs::PoseWithCovarianceStamped initPoseMsg;
    Poco::JSON::Object::Ptr positionData = req->getObject("position");
    initPoseMsg.pose.pose.position.x = (float)positionData->get("x");
    initPoseMsg.pose.pose.position.y = (float)positionData->get("y");
    initPoseMsg.pose.pose.position.z = (float)positionData->get("z");

    Poco::JSON::Object::Ptr orientationData = req->getObject("orientation");
    initPoseMsg.pose.pose.orientation.x = (float)orientationData->get("x");
    initPoseMsg.pose.pose.orientation.y = (float)orientationData->get("y");
    initPoseMsg.pose.pose.orientation.z = (float)orientationData->get("z");
    initPoseMsg.pose.pose.orientation.w = (float)orientationData->get("w");

    initpose_pub_.publish(initPoseMsg);

    bool correct_position = req->get("correct_position");

    if (correct_position) {
        initpose_correct_position_pub_.publish(std_msgs::String());
    }
    // 성공 응답
    ctx.ok(name());

    LOG_INFO("[ActionInitPose] 응답 완료");
}

}  // namespace NaviFra
