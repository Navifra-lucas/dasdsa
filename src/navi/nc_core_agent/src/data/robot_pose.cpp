#include "core_agent/core_agent.h"

#include <core_agent/data/robot_pose.h>

using namespace NaviFra;

const std::string RobotPose::KEY = "LidarMerger";

void RobotPose::update(const Position& position, const Orientation& orientation)
{
    std::lock_guard<std::mutex> lock(mutex_);
    position_ = position;
    orientation_ = orientation;
}

const Poco::JSON::Object::Ptr RobotPose::toObject()
{
    std::lock_guard<std::mutex> lock(mutex_);
    Poco::JSON::Object::Ptr robot_pose = new Poco::JSON::Object();
    Poco::JSON::Object position, orientation;
    RobotStatus::Ptr robot_status = InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY);
    if (position_.x == 0 && position_.y == 0 &&
        robot_status->isSLAM() != true)  // intialize pose 안됐을때 pose값이 0,0으로 들어가는 것 방지
    {
        return robot_pose;
    }
    position.set("x", position_.x);
    position.set("y", position_.y);
    robot_pose->set("position", position);

    orientation.set("x", orientation_.x);
    orientation.set("y", orientation_.y);
    orientation.set("z", orientation_.z);
    orientation.set("w", orientation_.w);
    robot_pose->set("orientation", orientation);

    return robot_pose;
}
