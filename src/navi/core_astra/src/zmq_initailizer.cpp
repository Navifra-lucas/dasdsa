#include "core_astra/initializer/zmq_initializer.h"
#include "core_astra/zmq_handler.h"
#include "util/logger.hpp"

#include <Poco/Delegate.h>
#include <Poco/ThreadPool.h>

#include <atomic>
#include <functional>

using namespace NaviFra;

void globalDispatcher(const void* sender, const std::string& msg)
{
    if (msg.find("status") != std::string::npos) {
        //  ROS_INFO_STREAM("[Dispatcher] Save status: " << msg);
        // 실제 status 저장 로직 추가
    }
    else {
        // ROS_INFO_STREAM("[Dispatcher] Log only: " << msg);
    }
}

ZMQInitializer::~ZMQInitializer()
{
    shutdown();
    // 소멸자에서 특별한 작업은 필요하지 않음
    LOG_INFO("ZMQInitializer destroyed");
}

void ZMQInitializer::pollerLoop()
{
    auto& handler = ZMQHandler::instance();
    while (running_) {
        std::string msg;
        if (handler.tryRecv(msg)) {
            // ROS_INFO_STREAM("[Dispatcher] ZMQ message: " << msg);
            handler.onMessage_.notify(this, msg);
        }
        Poco::Thread::sleep(10);
    }
}

void ZMQInitializer::initialize()
{
    auto& zmq_handler = ZMQHandler::instance();
    zmq_handler.init("5556", "5557", "5558");

    zmq_handler.onMessage_ += Poco::delegate(globalDispatcher);
    // Poller 실행
    running_.store(true);
    // RunnableAdapter를 이용해 멤버 함수 pollerLoop 실행
    Poco::RunnableAdapter<ZMQInitializer> runnable(*this, &ZMQInitializer::pollerLoop);
    Poco::ThreadPool::defaultPool().start(runnable);

    LOG_INFO("ZMQInitializer completed initialization (poller started)");
}

void ZMQInitializer::shutdown()
{
    // Poller 종료
    running_.store(false);
    Poco::ThreadPool::defaultPool().joinAll();

    auto& zmq_handler = ZMQHandler::instance();
    zmq_handler.onMessage_ -= Poco::delegate(globalDispatcher);
    // Poller 실행

    LOG_INFO("ZMQInitializer shutdown (poller stopped)");
}
