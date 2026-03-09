#include "core_astra/initializer/zmq_initializer.h"
#include "core_astra/message/astra_message.h"
#include "core_astra/zmq_handler.h"
#include "util/logger.hpp"

#include <Poco/Delegate.h>
#include <Poco/ThreadPool.h>

#include <atomic>
#include <functional>

using namespace NaviFra;

ZMQInitializer::~ZMQInitializer()
{
    shutdown();

}

void ZMQInitializer::pollerLoop()
{
    auto& handler = ZMQHandler::instance();
    while (running_) {
        std::string msg;
        if (handler.tryRecv(msg)) {
            LOG_INFO("[Dispatcher] ZMQ message:  %s", msg.c_str());
        }
        Poco::Thread::sleep(10);
    }
}

void ZMQInitializer::initialize()
{
    auto& zmq_handler = ZMQHandler::instance();
    zmq_handler.init("5556", "5557", "5558");

    zmq_handler.onMessageCollector_ += Poco::delegate(onMessageCollector);
    zmq_handler.onMessageRouterRequest_ += Poco::delegate(handleRouterRequest);

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
    auto& zmq_handler = ZMQHandler::instance();
    zmq_handler.onMessageCollector_ -= Poco::delegate(onMessageCollector);
    zmq_handler.onMessageRouterRequest_ -= Poco::delegate(handleRouterRequest);
    // Poller 실행

    LOG_INFO("ZMQInitializer shutdown (poller stopped)");
}

void ZMQInitializer::finalize()
{
    shutdown();
}
