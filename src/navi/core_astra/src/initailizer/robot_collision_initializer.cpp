#include "core_astra/initializer/robot_collision_initializer.h"

#include "core_agent/data/memory_repository.h"
#include "core_agent/data/robot_collision_info.h"
#include "core_agent/manager/publish_manager.h"
#include "core_astra/data/publish_channel.h"
#include "core_astra/zmq_handler.h"
#include "robot_collision.pb.h"
#include "util/logger.hpp"

using namespace NaviFra;

void RobotCollisionInitializer::initialize()
{
    // 로봇 충돌 정보 전송
    PublisherManager::instance().addChannel(
        (int)PUBLISH_CHANNEL::CHANNEL_STATUS_COLLISION,
        []() mutable {
            // 현재 충돌 정보 가져오기
            auto collision = InMemoryRepository::instance().get<RobotCollisionInfo>(RobotCollisionInfo::KEY);

            // Protobuf 메시지 생성
            core_astra::RobotCollisionInfo status;
            auto mutable_collision = status.mutable_collision();

            for (const auto& p : collision->getCollisionVector()) {
                auto pos = mutable_collision->Add();
                pos->set_x(p.x);
                pos->set_y(p.y);
                pos->set_z(0);  // Z는 현재 0으로 고정
            }

            // 직렬화
            std::string buffer;
            status.SerializeToString(&buffer);

            // ZMQ로 전송
            ZMQHandler::instance().send("robot_collision_info", buffer);
        },
        20, 0  // 주기: 20ms, 지연 없음
    );

    LOG_INFO("RobotCollisionInitializer completed initialization");
}
