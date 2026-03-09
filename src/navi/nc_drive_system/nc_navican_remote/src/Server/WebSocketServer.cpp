#include "nc_navican_remote/Server/WebSocketServer.h"

#include "nc_navican_remote/Utils/JsonUtils.h"
#include "util/logger.hpp"

#include <algorithm>
#include <chrono>
#include <thread>

using namespace NaviFra::NaviCAN::Remote;
using namespace std::chrono_literals;

void WebSocketServer::start()
{
    running_ = true;
    broadcast_enabled_ = true;
    doAccept();
    startBroadcastThread();
    NLOG(info) << "WebSocket Server started on port " << acceptor_.local_endpoint().port();
}

void WebSocketServer::stop()
{
    if (running_.exchange(false)) {
        broadcast_enabled_ = false;
        acceptor_.close();

        {
            std::scoped_lock lock(sessions_mutex_);
            sessions_.clear();
        }

        if (broadcast_thread_.joinable()) {
            broadcast_thread_.join();
        }

        NLOG(info) << "WebSocket Server stopped";
    }
}

void WebSocketServer::doAccept()
{
    acceptor_.async_accept([this](beast::error_code ec, tcp::socket socket) {
        if (!ec) {
            auto session = std::make_shared<WebSocketSession>(std::move(socket), navican_, this);
            session->start();
            addSession(session);
        }
        else {
            NLOG(severity_level::error) << "Accept error: " << ec.message();
        }

        if (running_) {
            doAccept();
        }
    });
}

void WebSocketServer::addSession(std::shared_ptr<WebSocketSession> session)
{
    std::scoped_lock lock(sessions_mutex_);
    sessions_.push_back(std::move(session));
    NLOG(info) << "WebSocket session added. Total sessions: " << sessions_.size();
}

void WebSocketServer::removeSession(std::shared_ptr<WebSocketSession> session)
{
    std::scoped_lock lock(sessions_mutex_);
    if (auto it = std::find(sessions_.begin(), sessions_.end(), session); it != sessions_.end()) {
        sessions_.erase(it);
        NLOG(info) << "WebSocket session removed. Total sessions: " << sessions_.size();
    }
}

void WebSocketServer::startBroadcastThread()
{
    broadcast_thread_ = std::thread([this]() {
        while (running_) {
            if (broadcast_enabled_) {
                broadcastRealtimeData();
            }
            std::this_thread::sleep_for(broadcast_interval_);
        }
    });
}

void WebSocketServer::broadcastRealtimeData()
{
    try {
        cleanupInactiveSessions();

        RealtimeDataPacket packet = createRealtimePacket();

        rapidjson::Document broadcast_data;
        broadcast_data.SetObject();
        auto& allocator = broadcast_data.GetAllocator();

        broadcast_data.AddMember("type", "realtime_broadcast", allocator);
        broadcast_data.AddMember("sequence_number", packet.sequence_number, allocator);
        broadcast_data.AddMember("timestamp", packet.timestamp, allocator);

        rapidjson::Value motors_array(rapidjson::kArrayType);
        for (const auto& motor : packet.motor_data) {
            rapidjson::Value motor_json(rapidjson::kObjectType);
            motor_json.AddMember("motorId", motor.motor_id, allocator);
            motor_json.AddMember("position", motor.position, allocator);
            motor_json.AddMember("speed", motor.speed, allocator);
            motor_json.AddMember("current", motor.current, allocator);
            motor_json.AddMember("voltage", motor.voltage, allocator);
            motor_json.AddMember("state", motor.state, allocator);
            motors_array.PushBack(motor_json, allocator);
        }
        broadcast_data.AddMember("motors", motors_array, allocator);

        const std::string broadcast_str = JsonUtils::jsonToString(broadcast_data);

        std::scoped_lock lock(sessions_mutex_);
        for (const auto& session : sessions_) {
            if (session && session->isOpen()) {
                session->send(broadcast_str);
            }
        }
    }
    catch (const std::exception& e) {
        NLOG(severity_level::error) << "Broadcast error: " << e.what();
    }
}

void WebSocketServer::cleanupInactiveSessions()
{
    std::scoped_lock lock(sessions_mutex_);
    const auto now = std::chrono::steady_clock::now();
    constexpr auto timeout = std::chrono::seconds(30);

    // remove_if : 조건에 맞는 아이템 맨 뒤로, 범위 조정, 삭제하지 않음
    auto it = std::remove_if(sessions_.begin(), sessions_.end(), [now, timeout](const auto& session) {
        if (!session->isOpen()) {
            return true;
        }
        const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - session->getLastActivity());
        return elapsed > timeout;
    });

    // 삭제
    sessions_.erase(it, sessions_.end());
}

RealtimeDataPacket WebSocketServer::createRealtimePacket()
{
    RealtimeDataPacket packet;
    packet.sequence_number = sequence_number_++;
    packet.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();

    // Get all motor IDs from the controllers map
    auto& controllers = navican_->getControllers();
    for (const auto& [motor_id, controller] : controllers) {
        MotorRealtimeData data;
        data.motor_id = motor_id;
        data.position = navican_->getPosition(motor_id);
        data.speed = navican_->getSpeed(motor_id);
        data.current = navican_->getCurrent(motor_id);
        data.voltage = navican_->getVoltage(motor_id);
        data.state = static_cast<int>(navican_->getState(motor_id));

        packet.motor_data.push_back(data);
    }

    return packet;
}
