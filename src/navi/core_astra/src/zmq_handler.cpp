#include "core_astra/zmq_handler.h"

#include "util/logger.hpp"

#include <zmq_addon.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <utility>

using namespace NaviFra;

ZMQHandler::ZMQHandler()
    : context_(1)
    , initialized_(false)
    , router_(context_, zmq::socket_type::router)
    , publisher_(context_, zmq::socket_type::pub)
    , collector_(context_, zmq::socket_type::pull)
    , pub_stop_(false)  // ★ 추가
{
    int linger = 0;
    int hwm = 10000;
    int sndtimeo = 100;
    int rcvtimeo = 0;
    int handover = 1;

    router_.setsockopt(ZMQ_LINGER, &linger, sizeof(linger));
    router_.setsockopt(ZMQ_SNDHWM, &hwm, sizeof(hwm));
    router_.setsockopt(ZMQ_RCVHWM, &hwm, sizeof(hwm));
    router_.setsockopt(ZMQ_SNDTIMEO, &sndtimeo, sizeof(sndtimeo));
    router_.setsockopt(ZMQ_RCVTIMEO, &rcvtimeo, sizeof(rcvtimeo));
    router_.setsockopt(ZMQ_ROUTER_HANDOVER, &handover, sizeof(handover));

    publisher_.setsockopt(ZMQ_LINGER, &linger, sizeof(linger));
    collector_.setsockopt(ZMQ_LINGER, &linger, sizeof(linger));
}

ZMQHandler::ZMQHandler(const std::string& snapshot_port, const std::string& pub_port, const std::string& pull_port)
    : ZMQHandler()
{
    init(snapshot_port, pub_port, pull_port);
}

ZMQHandler::~ZMQHandler()
{  // ★ 추가: 안전 종료
    stopPublisher();
    try {
        int zero = 0;
        publisher_.setsockopt(ZMQ_LINGER, &zero, sizeof(zero));
        publisher_.close();
    }
    catch (...) {
    }
    try {
        int zero = 0;
        router_.setsockopt(ZMQ_LINGER, &zero, sizeof(zero));
        router_.close();
        collector_.setsockopt(ZMQ_LINGER, &zero, sizeof(zero));
        collector_.close();
    }
    catch (...) {
    }
}

void ZMQHandler::init(const std::string& snapshot_port, const std::string& pub_port, const std::string& pull_port)
{
    if (initialized_) {
        LOG_WARNING("ZMQHandler already initialized");
        return;
    }

    router_.bind("tcp://*:" + snapshot_port);
    publisher_.bind("tcp://*:" + pub_port);
    collector_.bind("tcp://*:" + pull_port);

    initialized_ = true;
    LOG_INFO("ZMQHandler initialized with ports [ %s, %s, %s ]", snapshot_port.c_str(), pub_port.c_str(), pull_port.c_str());

    startPublisher();  // ★ 추가: 워커 시작
}

void ZMQHandler::startPublisher()  // ★ 추가
{
    if (pub_thread_.joinable())
        return;

    pub_stop_ = false;
    pub_thread_ = std::thread([this] { publisherWorker(); });
}

void ZMQHandler::stopPublisher()  // ★ 추가
{
    if (!pub_thread_.joinable())
        return;

    pub_stop_ = true;
    pub_cv_.notify_all();  // 대기 중 워커 깨우기
    try {
        // poll이 아니라 condvar 기반이라 푸시 불필요하지만,
        // 필요하다면 inproc 푸시로 깨울 수도 있음
    }
    catch (...) {
    }

    pub_thread_.join();
}

void ZMQHandler::publisherWorker()  // ★ 추가: 이 스레드만 publisher_ 사용
{
    // 퍼블리셔 소켓 추가 옵션 (연결 전 송신 방지/보호)
    int immediate = 1;
    int sndhwm = 10000;
    publisher_.setsockopt(ZMQ_IMMEDIATE, &immediate, sizeof(immediate));
    publisher_.setsockopt(ZMQ_SNDHWM, &sndhwm, sizeof(sndhwm));

    // 슬로우 조이너 보호(구독 전파 대기)
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    while (!pub_stop_) {
        std::pair<std::string, std::string> msg;
        {
            std::unique_lock<std::mutex> lk(pub_mtx_);
            pub_cv_.wait(lk, [&] { return pub_stop_ || !pub_q_.empty(); });
            if (pub_stop_)
                break;
            msg = std::move(pub_q_.front());
            pub_q_.pop_front();
        }

        // 실제 전송 (이 스레드만 소켓 접근)
        try {
            publisher_.send(zmq::buffer(msg.first), zmq::send_flags::sndmore);
            publisher_.send(zmq::buffer(msg.second), zmq::send_flags::none);
        }
        catch (const zmq::error_t& e) {
            LOG_ERROR("ZMQ publish failed: %s", e.what());
            // 필요 시 재시도/백오프 정책 추가 가능
        }
    }
}

void ZMQHandler::enqueuePublish(std::string topic, std::string buffer)  // ★ 추가: 큐 적재 API
{
    {
        std::lock_guard<std::mutex> lk(pub_mtx_);
        pub_q_.emplace_back(std::move(topic), std::move(buffer));
    }
    pub_cv_.notify_one();
}

void ZMQHandler::poll()
{
    zmq::pollitem_t items[] = {{static_cast<void*>(collector_), 0, ZMQ_POLLIN, 0}, {static_cast<void*>(router_), 0, ZMQ_POLLIN, 0}};
    zmq::poll(items, 2, 0);

    if (items[0].revents & ZMQ_POLLIN) {
        zmq::message_t msg;
        collector_.recv(&msg);
        onMessageCollector_.notify(this, std::string(static_cast<char*>(msg.data()), msg.size()));
    }
    if (items[1].revents & ZMQ_POLLIN) {
        std::string id, payload;
        if (recvRouterOnce(id, payload)) {
            RouterMsg ev{id, payload};
            onMessageRouterRequest_.notify(this, ev);
        }
    }
}

bool ZMQHandler::recvRouterOnce(std::string& identity, std::string& payload)
{
    std::vector<zmq::message_t> frames;

    auto ok = zmq::recv_multipart(router_, std::back_inserter(frames), zmq::recv_flags::dontwait);
    if (!ok || frames.size() < 2)
        return false;

    // REQ: [id][empty][payload], DEALER: [id][payload]
    identity.assign(static_cast<const char*>(frames[0].data()), frames[0].size());

    size_t idx = 1;
    if (frames.size() >= 3 && frames[1].size() == 0)
        idx = 2;

    payload.assign(static_cast<const char*>(frames[idx].data()), frames[idx].size());
    return true;
}

bool ZMQHandler::tryRecv(std::string& outMessage)
{
    poll();
    return false;  // kept for API compatibility
}

// ★ 변경: 실제 소켓 send 금지 → 큐 적재만 수행
void ZMQHandler::send(const std::string& topic, const std::string& buffer, std::function<void(const zmq::message_t&)> /*callback*/)
{
    try {
        enqueuePublish(topic, buffer);
    }
    catch (const std::exception& e) {
        LOG_ERROR("enqueuePublish failed: %s", e.what());
    }
}
