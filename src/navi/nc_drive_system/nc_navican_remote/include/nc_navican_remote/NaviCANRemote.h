#ifndef NAVIFRA_NAVICAN_REMOTE_H
#define NAVIFRA_NAVICAN_REMOTE_H

#include "nc_navican_remote/Server/WebSocketServer.h"
#include "nc_navican_remote/Utils/types.h"

#include <boost/asio.hpp>

#include <atomic>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>

namespace asio = boost::asio;

namespace NaviFra {
namespace NaviCAN {
namespace Remote {

// WebSocket 서버 매니저
class NaviCANRemote {
public:
    NaviCANRemote(short ws_port = 8888)
        : ws_port_{ws_port}
        , navican_{std::make_shared<NaviCANCore>()}
    {
    }
    ~NaviCANRemote() { finalize(); }

    bool initialize();
    void run();
    void finalize();

    NaviCANCore& getNaviCAN() { return *navican_; }

    // 브로드캐스트 제어
    void enableRealtimeBroadcast(bool enable);
    void setRealtimeBroadcastRate(int hz);

private:
    const short ws_port_;
    std::shared_ptr<NaviCANCore> navican_;

    asio::io_context ws_io_context_;
    std::thread ws_thread_;
    std::unique_ptr<WebSocketServer> ws_server_;

    std::atomic<bool> running_{false};
};

}  // namespace Remote
}  // namespace NaviCAN
}  // namespace NaviFra

#endif  // NAVIFRA_NAVICAN_REMOTE_H