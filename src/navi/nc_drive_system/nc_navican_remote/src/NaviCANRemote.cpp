#include "nc_navican_remote/NaviCANRemote.h"

#include "util/logger.hpp"

#include <chrono>
#include <sstream>

using namespace NaviFra;
using namespace NaviFra::NaviCAN::Remote;
using namespace std::chrono_literals;

bool NaviCANRemote::initialize()
{
    try {
        if (!navican_->initialize()) {
            NLOG(severity_level::error) << "Failed to initialize NaviCANCore";
            return false;
        }

        ws_server_ = std::make_unique<WebSocketServer>(ws_io_context_, ws_port_, navican_);
        ws_server_->start();

        NLOG(info) << "NaviCANRemote initialized successfully";
        NLOG(info) << "WebSocket Server: port " << ws_port_;
        return true;
    }
    catch (const std::exception& e) {
        NLOG(severity_level::error) << "Failed to initialize NaviCANRemote: " << e.what();
        return false;
    }
}

void NaviCANRemote::run()
{
    running_ = true;

    ws_thread_ = std::thread([this]() {
        try {
            ws_io_context_.run();
        }
        catch (const std::exception& e) {
            NLOG(severity_level::error) << "WebSocket server thread error: " << e.what();
        }
    });

    NLOG(info) << "NaviCANRemote is running";
}

void NaviCANRemote::finalize()
{
    if (running_.exchange(false)) {
        if (ws_server_) {
            ws_server_->stop();
        }

        ws_io_context_.stop();

        if (ws_thread_.joinable()) {
            ws_thread_.join();
        }

        if (navican_) {
            navican_->finalize();
        }

        NLOG(info) << "NaviCANRemote finalized";
    }
}

void NaviCANRemote::enableRealtimeBroadcast(bool enable)
{
    if (ws_server_) {
        ws_server_->broadcast_enabled_ = enable;
        NLOG(info) << "Realtime broadcast " << (enable ? "enabled" : "disabled");
    }
}

void NaviCANRemote::setRealtimeBroadcastRate(int hz)
{
    if (hz < 1 || hz > 100) {
        NLOG(warning) << "Invalid broadcast rate: " << hz << "Hz. Must be 1-100Hz.";
        return;
    }

    if (ws_server_) {
        const auto interval_ms = 1000 / hz;
        ws_server_->broadcast_interval_ = std::chrono::milliseconds(interval_ms);
        NLOG(info) << "Realtime broadcast rate set to " << hz << "Hz (" << interval_ms << "ms interval)";
    }
}