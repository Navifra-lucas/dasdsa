#include "core_astra/initializer/robot_pose_initailizer.h"

#include "core_agent/data/memory_repository.h"
#include "core_agent/data/robot_pose.h"
#include "core_agent/manager/publish_manager.h"
#include "core_astra/data/publish_channel.h"
#include "core_astra/zmq_handler.h"
#include "robot_pose.pb.h"
#include "util/logger.hpp"

using namespace NaviFra;

RobotPoseInitializer::~RobotPoseInitializer()
{
}

void RobotPoseInitializer::initialize()
{
    // 자신의 위치와 속도 정보만 전달 함
    PublisherManager::instance().addChannel(
        (int)PUBLISH_CHANNEL::CHANNEL_STATUS_POSE,
        []() mutable {
            auto robot_pose = InMemoryRepository::instance().get<RobotPose>(RobotPose::KEY);
            core_astra::RobotPose pose;
            pose.mutable_position()->set_x(robot_pose->getPosition().x);
            pose.mutable_position()->set_y(robot_pose->getPosition().y);
            pose.mutable_orientation()->set_x(robot_pose->getOrientation().x);
            pose.mutable_orientation()->set_y(robot_pose->getOrientation().y);
            pose.mutable_orientation()->set_z(robot_pose->getOrientation().z);
            pose.mutable_orientation()->set_w(robot_pose->getOrientation().w);

            std::string buffer;
            pose.SerializeToString(&buffer);

            ZMQHandler::instance().send("robot_pose", buffer);
        },
        20, 0);

    LOG_INFO("RobotPoseInitializer completed initialization");
}