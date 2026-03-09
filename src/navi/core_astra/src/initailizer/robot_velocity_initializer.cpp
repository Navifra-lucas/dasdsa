#include "core_astra/initializer/robot_velocity_initializer.h"

#include "core_agent/data/memory_repository.h"
#include "core_agent/data/robot_info.h"  // 속도 데이터 제공 객체 (예: RobotInfo)
#include "core_agent/manager/publish_manager.h"
#include "core_astra/data/publish_channel.h"
#include "core_astra/zmq_handler.h"
#include "robot_velocity.pb.h"
#include "util/logger.hpp"

using namespace NaviFra;

void RobotVelocityInitializer::initialize()
{
    // 로봇 속도 정보 전송
    PublisherManager::instance().addChannel(
        (int)PUBLISH_CHANNEL::CHANNEL_STATUS_VELOCITY,
        []() mutable {
            auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);

            auto robotVelocity = robotInfo->getVelocity();

            core_astra::RobotVelocity velocity;
            velocity.set_linear_x(robotVelocity[0]);
            velocity.set_linear_y(robotVelocity[0]);
            velocity.set_angular_z(robotVelocity[0]);

            std::string buffer;
            velocity.SerializeToString(&buffer);

            ZMQHandler::instance().send("robot_velocity", buffer);
        },
        20, 0);  // 주기 20ms, 지연 0

    LOG_INFO("RobotVelocityInitializer completed initialization");
}
