#ifndef NAVIFRA_NAVICAN_REMOTE_WEBSOCKET_SERVER
#define NAVIFRA_NAVICAN_REMOTE_WEBSOCKET_SERVER

#include "NaviCAN/NaviCANCore.h"
#include "nc_navican_remote/Server/WebSocketSession.h"
#include "nc_navican_remote/Utils/types.h"

#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;

namespace NaviFra {
namespace NaviCAN {
namespace Remote {

class NaviCANRemote;

// WebSocket 서버 (실시간 + 중요 명령 모두 처리)
class WebSocketServer {
public:
    WebSocketServer(net::io_context& io_context, short port, std::shared_ptr<NaviCANCore> navican_)
        : io_context_(io_context)
        , acceptor_(io_context, tcp::endpoint(tcp::v4(), port))
        , navican_(navican_)
    {
    }

    ~WebSocketServer() { stop(); }

    void start();
    void stop();

    // 브로드캐스트 용
    std::atomic<bool> broadcast_enabled_{false};
    std::chrono::milliseconds broadcast_interval_{100};

    // 세션 관리
    void addSession(std::shared_ptr<WebSocketSession> session);
    void removeSession(std::shared_ptr<WebSocketSession> session);

private:
    void doAccept();
    void broadcastRealtimeData();
    void startBroadcastThread();
    void cleanupInactiveSessions();

    net::io_context& io_context_;
    tcp::acceptor acceptor_;
    std::shared_ptr<NaviCANCore> navican_;
    std::atomic<bool> running_{false};

    // 세션 관리
    std::mutex sessions_mutex_;
    std::vector<std::shared_ptr<WebSocketSession>> sessions_;

    // 브로드캐스트
    std::thread broadcast_thread_;
    uint32_t sequence_number_{0};

    RealtimeDataPacket createRealtimePacket();

    friend class NaviCANRemote;
};

}  // namespace Remote
}  // namespace NaviCAN
}  // namespace NaviFra

#endif
