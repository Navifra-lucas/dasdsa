#ifndef NAVIFRA_NAVICAN_REMOTE_WEBSOCKET_SESSION
#define NAVIFRA_NAVICAN_REMOTE_WEBSOCKET_SESSION

#include "NaviCAN/NaviCANCore.h"
#include "nc_navican_remote/Utils/JsonUtils.h"

#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;

namespace NaviFra {
namespace NaviCAN {
namespace Remote {

class WebSocketServer;

// WebSocket 세션
class WebSocketSession : public std::enable_shared_from_this<WebSocketSession> {
public:
    WebSocketSession(tcp::socket socket, std::shared_ptr<NaviCANCore> navican, WebSocketServer* server);
    ~WebSocketSession();

    void start();
    void send(const std::string& message);
    bool isOpen() const;
    std::chrono::steady_clock::time_point getLastActivity() const { return last_activity_; }

private:
    void doRead();
    void doWrite();
    void handleRequest(const std::string& request);
    rapidjson::Document processCommand(const rapidjson::Document& command);

    // 헬퍼 함수들
    uint8_t extractMotorId(const rapidjson::Value* params) const;
    rapidjson::Document createResultResponse(bool result) const;
    rapidjson::Document createMotorDataResponse(uint8_t motorId, const std::string& key, double value) const;

    // 제어 명령 핸들러 (Control Commands) - 단일 모터 제어
    rapidjson::Document handleSetTarget(const rapidjson::Value* params);
    rapidjson::Document handleEnable(const rapidjson::Value* params);
    rapidjson::Document handleDisable(const rapidjson::Value* params);
    rapidjson::Document handleShutdown(const rapidjson::Value* params);
    rapidjson::Document handleResetError(const rapidjson::Value* params);

    // 시스템 명령 핸들러 (System Commands) - 전체 모터 제어, 시스템 레벨
    rapidjson::Document handleEnableAll(const rapidjson::Value* params);
    rapidjson::Document handleDisableAll(const rapidjson::Value* params);
    rapidjson::Document handleShutdownAll(const rapidjson::Value* params);
    rapidjson::Document handleResetErrorAll(const rapidjson::Value* params);
    rapidjson::Document handlePresetEncoderAll(const rapidjson::Value* params);
    rapidjson::Document handleEmergencyStop(const rapidjson::Value* params);
    rapidjson::Document handleGetCanBusState(const rapidjson::Value* params);

    // 조회 명령 핸들러 (Query Commands) - 상태 조회, 데이터 읽기
    rapidjson::Document handleGetPosition(const rapidjson::Value* params);
    rapidjson::Document handleGetSpeed(const rapidjson::Value* params);
    rapidjson::Document handleGetCurrent(const rapidjson::Value* params);
    rapidjson::Document handleGetVoltage(const rapidjson::Value* params);
    rapidjson::Document handleGetMultiplePositions(const rapidjson::Value* params);
    rapidjson::Document handleGetMultipleSpeeds(const rapidjson::Value* params);
    rapidjson::Document handleGetAllMotorData(const rapidjson::Value* params);

    // 브로드캐스트 관련 핸들러 (Broadcast) - 실시간 데이터 구독
    rapidjson::Document handleSubscribeRealtime(const rapidjson::Value* params);
    rapidjson::Document handleUnsubscribeRealtime(const rapidjson::Value* params);

    websocket::stream<tcp::socket> ws_;
    std::shared_ptr<NaviCANCore> navican_;
    WebSocketServer* server_;
    beast::flat_buffer buffer_;
    std::vector<std::string> write_queue_;
    bool writing_{false};
    bool subscribed_to_realtime_{false};
    std::chrono::steady_clock::time_point last_activity_;

    using CommandHandler = std::function<rapidjson::Document(const rapidjson::Value*)>;
    std::unordered_map<std::string, CommandHandler> command_handlers_;

    void initializeCommandHandlers();
};

}  // namespace Remote
}  // namespace NaviCAN
}  // namespace NaviFra

#endif
