#include "nc_navican_remote/Server/WebSocketSession.h"

#include "nc_navican_remote/Server/WebSocketServer.h"
#include "nc_navican_remote/Utils/CommandClassifier.h"
#include "nc_navican_remote/Utils/JsonUtils.h"
#include "nc_navican_remote/Utils/types.h"
#include "util/logger.hpp"

using namespace NaviFra::NaviCAN::Remote;

WebSocketSession::WebSocketSession(tcp::socket socket, std::shared_ptr<NaviCANCore> navican, WebSocketServer* server)
    : ws_(std::move(socket))
    , navican_(navican)
    , server_(server)
    , last_activity_(std::chrono::steady_clock::now())
{
    initializeCommandHandlers();
}

WebSocketSession::~WebSocketSession()
{
    if (server_) {
        server_->removeSession(shared_from_this());
    }
}

void WebSocketSession::start()
{
    ws_.async_accept([self = shared_from_this()](beast::error_code ec) {
        if (!ec) {
            NLOG(info) << "WebSocket connection accepted";
            self->doRead();
        }
        else {
            NLOG(severity_level::error) << "WebSocket accept error: " << ec.message();
        }
    });
}

void WebSocketSession::doRead()
{
    ws_.async_read(buffer_, [self = shared_from_this()](beast::error_code ec, std::size_t bytes_transferred) {
        if (!ec) {
            self->last_activity_ = std::chrono::steady_clock::now();
            std::string request = beast::buffers_to_string(self->buffer_.data());
            self->buffer_.consume(bytes_transferred);
            self->handleRequest(request);
            self->doRead();
        }
        else if (ec != websocket::error::closed) {
            NLOG(severity_level::error) << "WebSocket read error: " << ec.message();
        }
    });
}

void WebSocketSession::send(const std::string& message)
{
    net::post(ws_.get_executor(), [self = shared_from_this(), message]() {
        self->write_queue_.push_back(message);
        if (self->write_queue_.size() == 1) {
            self->doWrite();
        }
    });
}

void WebSocketSession::doWrite()
{
    if (write_queue_.empty()) {
        return;
    }

    ws_.async_write(net::buffer(write_queue_.front()), [self = shared_from_this()](beast::error_code ec, std::size_t /*bytes_transferred*/) {
        if (!ec) {
            self->write_queue_.erase(self->write_queue_.begin());
            if (!self->write_queue_.empty()) {
                self->doWrite();
            }
        }
        else {
            NLOG(severity_level::error) << "WebSocket write error: " << ec.message();
        }
    });
}

bool WebSocketSession::isOpen() const
{
    return ws_.is_open();
}

void WebSocketSession::handleRequest(const std::string& request)
{
    try {
        const auto command_json = JsonUtils::stringToJson(request);
        auto response = processCommand(command_json);
        send(JsonUtils::jsonToString(response));
    }
    catch (const std::exception& e) {
        auto error_response = JsonUtils::createErrorResponse(std::string("Parse error: ") + e.what());
        send(JsonUtils::jsonToString(error_response));
    }
}

rapidjson::Document WebSocketSession::processCommand(const rapidjson::Document& command)
{
    try {
        if (!JsonUtils::hasField(command, "command")) {
            NLOG(info) << "No command field in request";
            return JsonUtils::createErrorResponse("No command specified");
        }

        const auto cmd = JsonUtils::safeGetString(command, "command");
        if (const auto handler_it = command_handlers_.find(cmd); handler_it != command_handlers_.end()) {
            // Get params as a Value reference (or nullptr if not exists)
            const rapidjson::Value* params = nullptr;
            if (JsonUtils::hasField(command, "params") && command["params"].IsObject()) {
                params = &command["params"];
            }
            return handler_it->second(params);
        }

        return JsonUtils::createErrorResponse("Unknown command: " + cmd);
    }
    catch (const std::exception& e) {
        return JsonUtils::createErrorResponse(std::string("Command processing error: ") + e.what());
    }
}

// ============================================================================
// 헬퍼 함수들
// ============================================================================

uint8_t WebSocketSession::extractMotorId(const rapidjson::Value* params) const
{
    return params ? JsonUtils::safeGetUint8(*params, "motorId", 0) : uint8_t{0};
}

rapidjson::Document WebSocketSession::createResultResponse(bool result) const
{
    rapidjson::Document data;
    data.SetObject();
    data.AddMember("result", result, data.GetAllocator());
    return JsonUtils::createSuccessResponse(&data);
}

rapidjson::Document WebSocketSession::createMotorDataResponse(uint8_t motorId, const std::string& key, double value) const
{
    rapidjson::Document data;
    data.SetObject();
    auto& allocator = data.GetAllocator();

    rapidjson::Value keyValue(key.c_str(), key.length(), allocator);
    data.AddMember(keyValue, value, allocator);
    data.AddMember("motorId", motorId, allocator);
    data.AddMember("timestamp",
                   std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::steady_clock::now().time_since_epoch()).count(),
                   allocator);

    return JsonUtils::createSuccessResponse(&data);
}

// ============================================================================
// 명령 핸들러 초기화
// ============================================================================

void WebSocketSession::initializeCommandHandlers()
{
    // 제어 명령 (Control Commands) - 단일 모터 제어
    command_handlers_["setTarget"] = [this](const rapidjson::Value* params) { return handleSetTarget(params); };
    command_handlers_["enable"] = [this](const rapidjson::Value* params) { return handleEnable(params); };
    command_handlers_["disable"] = [this](const rapidjson::Value* params) { return handleDisable(params); };
    command_handlers_["shutdown"] = [this](const rapidjson::Value* params) { return handleShutdown(params); };
    command_handlers_["resetError"] = [this](const rapidjson::Value* params) { return handleResetError(params); };

    // 시스템 명령 (System Commands) - 전체 모터 제어, 시스템 레벨
    command_handlers_["enableAll"] = [this](const rapidjson::Value* params) { return handleEnableAll(params); };
    command_handlers_["disableAll"] = [this](const rapidjson::Value* params) { return handleDisableAll(params); };
    command_handlers_["shutdownAll"] = [this](const rapidjson::Value* params) { return handleShutdownAll(params); };
    command_handlers_["resetErrorAll"] = [this](const rapidjson::Value* params) { return handleResetErrorAll(params); };
    command_handlers_["presetEncoderAll"] = [this](const rapidjson::Value* params) { return handlePresetEncoderAll(params); };
    command_handlers_["emergencyStop"] = [this](const rapidjson::Value* params) { return handleEmergencyStop(params); };
    command_handlers_["getCanBusState"] = [this](const rapidjson::Value* params) { return handleGetCanBusState(params); };

    // 조회 명령 (Query Commands) - 상태 조회, 데이터 읽기
    command_handlers_["getPosition"] = [this](const rapidjson::Value* params) { return handleGetPosition(params); };
    command_handlers_["getSpeed"] = [this](const rapidjson::Value* params) { return handleGetSpeed(params); };
    command_handlers_["getCurrent"] = [this](const rapidjson::Value* params) { return handleGetCurrent(params); };
    command_handlers_["getVoltage"] = [this](const rapidjson::Value* params) { return handleGetVoltage(params); };
    command_handlers_["getMultiplePositions"] = [this](const rapidjson::Value* params) { return handleGetMultiplePositions(params); };
    command_handlers_["getMultipleSpeeds"] = [this](const rapidjson::Value* params) { return handleGetMultipleSpeeds(params); };
    command_handlers_["getAllMotorData"] = [this](const rapidjson::Value* params) { return handleGetAllMotorData(params); };

    // 브로드캐스트 관련 (Broadcast) - 실시간 데이터 구독
    command_handlers_["subscribeRealtime"] = [this](const rapidjson::Value* params) { return handleSubscribeRealtime(params); };
    command_handlers_["unsubscribeRealtime"] = [this](const rapidjson::Value* params) { return handleUnsubscribeRealtime(params); };
}

// ============================================================================
// 제어 명령 핸들러 (Control Commands)
// ============================================================================

rapidjson::Document WebSocketSession::handleSetTarget(const rapidjson::Value* params)
{
    try {
        const auto motorId = params ? JsonUtils::safeGetUint8(*params, "motorId", 0) : uint8_t{0};
        const auto value = params ? JsonUtils::safeGetDouble(*params, "value", 0.0) : 0.0;

        navican_->setTarget(motorId, value);
        NLOG(info) << "WS: Set target Motor " << int(motorId) << " = " << value;
        return JsonUtils::createSuccessResponse();
    }
    catch (const std::exception& e) {
        return JsonUtils::createErrorResponse(e.what());
    }
}

rapidjson::Document WebSocketSession::handleEnable(const rapidjson::Value* params)
{
    try {
        const auto motorId = extractMotorId(params);
        const auto result = navican_->enable(motorId);
        NLOG(info) << "WS: Enable Motor " << int(motorId) << " = " << (result ? "OK" : "FAIL");
        return createResultResponse(result);
    }
    catch (const std::exception& e) {
        return JsonUtils::createErrorResponse(e.what());
    }
}

rapidjson::Document WebSocketSession::handleDisable(const rapidjson::Value* params)
{
    try {
        const auto motorId = extractMotorId(params);
        const auto result = navican_->disable(motorId);
        NLOG(info) << "WS: Disable Motor " << int(motorId) << " = " << (result ? "OK" : "FAIL");
        return createResultResponse(result);
    }
    catch (const std::exception& e) {
        return JsonUtils::createErrorResponse(e.what());
    }
}

rapidjson::Document WebSocketSession::handleShutdown(const rapidjson::Value* params)
{
    try {
        const auto motorId = extractMotorId(params);
        const auto result = navican_->shutdown(motorId);
        NLOG(warning) << "WS: Shutdown Motor " << int(motorId) << " = " << (result ? "OK" : "FAIL");
        return createResultResponse(result);
    }
    catch (const std::exception& e) {
        return JsonUtils::createErrorResponse(e.what());
    }
}

rapidjson::Document WebSocketSession::handleResetError(const rapidjson::Value* params)
{
    try {
        const auto motorId = extractMotorId(params);
        const auto result = navican_->resetError(motorId);
        NLOG(info) << "WS: Reset Error Motor " << int(motorId) << " = " << (result ? "OK" : "FAIL");
        return createResultResponse(result);
    }
    catch (const std::exception& e) {
        return JsonUtils::createErrorResponse(e.what());
    }
}

// ============================================================================
// 시스템 명령 핸들러 (System Commands)
// ============================================================================

rapidjson::Document WebSocketSession::handleEnableAll([[maybe_unused]] const rapidjson::Value* params)
{
    try {
        const auto result = navican_->enableAll();
        NLOG(info) << "WS: Enable All = " << (result ? "OK" : "FAIL");
        return createResultResponse(result);
    }
    catch (const std::exception& e) {
        return JsonUtils::createErrorResponse(e.what());
    }
}

rapidjson::Document WebSocketSession::handleDisableAll([[maybe_unused]] const rapidjson::Value* params)
{
    try {
        const auto result = navican_->disableAll();
        NLOG(info) << "WS: Disable All = " << (result ? "OK" : "FAIL");
        return createResultResponse(result);
    }
    catch (const std::exception& e) {
        return JsonUtils::createErrorResponse(e.what());
    }
}

rapidjson::Document WebSocketSession::handleShutdownAll([[maybe_unused]] const rapidjson::Value* params)
{
    try {
        const auto result = navican_->shutdownAll();
        NLOG(warning) << "WS: Shutdown All = " << (result ? "OK" : "FAIL");
        return createResultResponse(result);
    }
    catch (const std::exception& e) {
        return JsonUtils::createErrorResponse(e.what());
    }
}

rapidjson::Document WebSocketSession::handleResetErrorAll([[maybe_unused]] const rapidjson::Value* params)
{
    try {
        const auto result = navican_->resetErrorAll();
        NLOG(info) << "WS: Reset Error All = " << (result ? "OK" : "FAIL");
        return createResultResponse(result);
    }
    catch (const std::exception& e) {
        return JsonUtils::createErrorResponse(e.what());
    }
}

rapidjson::Document WebSocketSession::handlePresetEncoderAll([[maybe_unused]] const rapidjson::Value* params)
{
    try {
        const auto result = navican_->presetEncoderAll();
        NLOG(info) << "WS: Preset Encoder All = " << (result ? "OK" : "FAIL");
        return createResultResponse(result);
    }
    catch (const std::exception& e) {
        return JsonUtils::createErrorResponse(e.what());
    }
}

rapidjson::Document WebSocketSession::handleEmergencyStop([[maybe_unused]] const rapidjson::Value* params)
{
    try {
        const auto result = navican_->disableAll();
        NLOG(warning) << "WS: EMERGENCY STOP = " << (result ? "OK" : "FAIL");

        rapidjson::Document data;
        data.SetObject();
        auto& allocator = data.GetAllocator();
        data.AddMember("result", result, allocator);
        rapidjson::Value msg("Emergency stop executed", allocator);
        data.AddMember("message", msg, allocator);
        return JsonUtils::createSuccessResponse(&data);
    }
    catch (const std::exception& e) {
        return JsonUtils::createErrorResponse(e.what());
    }
}

rapidjson::Document WebSocketSession::handleGetCanBusState([[maybe_unused]] const rapidjson::Value* params)
{
    try {
        rapidjson::Document data;
        data.SetObject();
        data.AddMember("canBusState", navican_->getCanBusState(), data.GetAllocator());
        return JsonUtils::createSuccessResponse(&data);
    }
    catch (const std::exception& e) {
        return JsonUtils::createErrorResponse(e.what());
    }
}

// ============================================================================
// 브로드캐스트 관련 핸들러 (Broadcast)
// ============================================================================

rapidjson::Document WebSocketSession::handleSubscribeRealtime([[maybe_unused]] const rapidjson::Value* params)
{
    subscribed_to_realtime_ = true;
    auto response = JsonUtils::createSuccessResponse();
    auto& allocator = response.GetAllocator();
    rapidjson::Value msg("Subscribed to realtime data", allocator);
    response.AddMember("message", msg, allocator);
    NLOG(info) << "WS: Client subscribed to realtime data";
    return response;
}

rapidjson::Document WebSocketSession::handleUnsubscribeRealtime([[maybe_unused]] const rapidjson::Value* params)
{
    subscribed_to_realtime_ = false;
    auto response = JsonUtils::createSuccessResponse();
    auto& allocator = response.GetAllocator();
    rapidjson::Value msg("Unsubscribed from realtime data", allocator);
    response.AddMember("message", msg, allocator);
    NLOG(info) << "WS: Client unsubscribed from realtime data";
    return response;
}

// ============================================================================
// 조회 명령 핸들러 (Query Commands)
// ============================================================================

rapidjson::Document WebSocketSession::handleGetPosition(const rapidjson::Value* params)
{
    try {
        const auto motorId = extractMotorId(params);
        return createMotorDataResponse(motorId, "position", navican_->getPosition(motorId));
    }
    catch (const std::exception& e) {
        return JsonUtils::createErrorResponse(e.what());
    }
}

rapidjson::Document WebSocketSession::handleGetSpeed(const rapidjson::Value* params)
{
    try {
        const auto motorId = extractMotorId(params);
        return createMotorDataResponse(motorId, "speed", navican_->getSpeed(motorId));
    }
    catch (const std::exception& e) {
        return JsonUtils::createErrorResponse(e.what());
    }
}

rapidjson::Document WebSocketSession::handleGetCurrent(const rapidjson::Value* params)
{
    try {
        const auto motorId = extractMotorId(params);
        return createMotorDataResponse(motorId, "current", navican_->getCurrent(motorId));
    }
    catch (const std::exception& e) {
        return JsonUtils::createErrorResponse(e.what());
    }
}

rapidjson::Document WebSocketSession::handleGetVoltage(const rapidjson::Value* params)
{
    try {
        const auto motorId = extractMotorId(params);
        return createMotorDataResponse(motorId, "voltage", navican_->getVoltage(motorId));
    }
    catch (const std::exception& e) {
        return JsonUtils::createErrorResponse(e.what());
    }
}

rapidjson::Document WebSocketSession::handleGetMultiplePositions(const rapidjson::Value* params)
{
    try {
        rapidjson::Document data;
        data.SetObject();
        auto& allocator = data.GetAllocator();

        rapidjson::Value positions_array(rapidjson::kArrayType);

        if (params && JsonUtils::hasField(*params, "motorIds") && (*params)["motorIds"].IsArray()) {
            const rapidjson::Value& motorIds = (*params)["motorIds"];
            for (rapidjson::SizeType i = 0; i < motorIds.Size(); i++) {
                if (motorIds[i].IsUint()) {
                    uint8_t motorId = static_cast<uint8_t>(motorIds[i].GetUint());
                    double position = navican_->getPosition(motorId);

                    rapidjson::Value motor_pos(rapidjson::kObjectType);
                    motor_pos.AddMember("motorId", motorId, allocator);
                    motor_pos.AddMember("position", position, allocator);
                    positions_array.PushBack(motor_pos, allocator);
                }
            }
        }

        data.AddMember("positions", positions_array, allocator);
        data.AddMember("timestamp",
                       std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(),
                       allocator);

        return JsonUtils::createSuccessResponse(&data);
    }
    catch (const std::exception& e) {
        return JsonUtils::createErrorResponse(e.what());
    }
}

rapidjson::Document WebSocketSession::handleGetMultipleSpeeds(const rapidjson::Value* params)
{
    try {
        rapidjson::Document data;
        data.SetObject();
        auto& allocator = data.GetAllocator();

        rapidjson::Value speeds_array(rapidjson::kArrayType);

        if (params && JsonUtils::hasField(*params, "motorIds") && (*params)["motorIds"].IsArray()) {
            const rapidjson::Value& motorIds = (*params)["motorIds"];
            for (rapidjson::SizeType i = 0; i < motorIds.Size(); i++) {
                if (motorIds[i].IsUint()) {
                    uint8_t motorId = static_cast<uint8_t>(motorIds[i].GetUint());
                    double speed = navican_->getSpeed(motorId);

                    rapidjson::Value motor_speed(rapidjson::kObjectType);
                    motor_speed.AddMember("motorId", motorId, allocator);
                    motor_speed.AddMember("speed", speed, allocator);
                    speeds_array.PushBack(motor_speed, allocator);
                }
            }
        }

        data.AddMember("speeds", speeds_array, allocator);
        data.AddMember("timestamp",
                       std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(),
                       allocator);

        return JsonUtils::createSuccessResponse(&data);
    }
    catch (const std::exception& e) {
        return JsonUtils::createErrorResponse(e.what());
    }
}

rapidjson::Document WebSocketSession::handleGetAllMotorData(const rapidjson::Value* params)
{
    try {
        rapidjson::Document data;
        data.SetObject();
        auto& allocator = data.GetAllocator();

        rapidjson::Value motors_array(rapidjson::kArrayType);
        auto& controllers = navican_->getControllers();

        for (const auto& [motor_id, controller] : controllers) {
            rapidjson::Value motor_data(rapidjson::kObjectType);
            motor_data.AddMember("motorId", motor_id, allocator);
            motor_data.AddMember("position", navican_->getPosition(motor_id), allocator);
            motor_data.AddMember("speed", navican_->getSpeed(motor_id), allocator);
            motor_data.AddMember("current", navican_->getCurrent(motor_id), allocator);
            motor_data.AddMember("voltage", navican_->getVoltage(motor_id), allocator);
            motor_data.AddMember("state", static_cast<int>(navican_->getState(motor_id)), allocator);

            motors_array.PushBack(motor_data, allocator);
        }

        data.AddMember("motors", motors_array, allocator);
        data.AddMember("timestamp",
                       std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(),
                       allocator);

        return JsonUtils::createSuccessResponse(&data);
    }
    catch (const std::exception& e) {
        return JsonUtils::createErrorResponse(e.what());
    }
}
