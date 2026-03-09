#pragma once
#include <Poco/BasicEvent.h>
#include <zmq.hpp>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace NaviFra {

// Router 이벤트에서 넘길 구조체
struct RouterMsg {
    std::string identity;
    std::string payload;
};

class ZMQHandler {
public:
    static ZMQHandler& instance()
    {
        static ZMQHandler inst;
        return inst;
    }

    ZMQHandler();
    ZMQHandler(const std::string& snapshot_port, const std::string& pub_port, const std::string& pull_port);
    ~ZMQHandler();  // 전용 송신 스레드/소켓 안전 종료

    // 한 번만 호출
    void init(const std::string& snapshot_port, const std::string& pub_port, const std::string& pull_port);

    // I/O 폴링 (router/collector 전용, 소켓 생성 스레드에서 호출)
    void poll();

    bool tryRecv(std::string& outMessage);
    bool recvRouterOnce(std::string& outIdentity, std::string& outPayload);

    // 퍼블리시 요청 (큐에 적재; 실제 전송은 내부 워커 스레드가 수행)
    void send(const std::string& topic, const std::string& buffer, std::function<void(const zmq::message_t&)> callback = nullptr);

    // 소켓 접근자 (호환 목적으로 유지)
    // 주의: ZeroMQ 소켓은 "한 소켓 = 한 스레드" 규칙. publisher()를 외부 스레드에서 직접 사용 금지.
    zmq::socket_t& router() { return router_; }
    // zmq::socket_t& publisher() { return publisher_; }
    zmq::socket_t& collector() { return collector_; }

    Poco::BasicEvent<RouterMsg> onMessageRouterRequest_;
    Poco::BasicEvent<const std::string> onMessageCollector_;

    // 복사/이동 금지 (싱글턴)
    ZMQHandler(const ZMQHandler&) = delete;
    ZMQHandler& operator=(const ZMQHandler&) = delete;
    ZMQHandler(ZMQHandler&&) = delete;
    ZMQHandler& operator=(ZMQHandler&&) = delete;

private:
    // --- 내부 전용: 전용 송신 스레드 + 안전 큐(A안) ---
    void startPublisher();  // init()에서 호출
    void stopPublisher();  // 소멸자에서 호출
    void publisherWorker();  // 이 스레드만 publisher_ 소켓을 사용
    void enqueuePublish(std::string topic, std::string buffer);

private:
    zmq::context_t context_;
    std::atomic<bool> initialized_;
    zmq::socket_t router_;
    zmq::socket_t publisher_;
    zmq::socket_t collector_;

    // A안용 멤버
    std::mutex pub_mtx_;
    std::condition_variable pub_cv_;
    std::deque<std::pair<std::string, std::string>> pub_q_;
    std::thread pub_thread_;
    std::atomic<bool> pub_stop_{false};

    uint64_t sequence_{0};
};

}  // namespace NaviFra
