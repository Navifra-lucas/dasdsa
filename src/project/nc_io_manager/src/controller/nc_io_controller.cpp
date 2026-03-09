#include "nc_io_manager/controller/nc_io_controller.h"

#include "util/logger.hpp"

#include <chrono>

namespace NaviFra {

IOController::IOController(ros::NodeHandle nh)
    : nh_(nh)
    , running_(false)
    , max_retry_count_(5)
    , current_retry_count_(0)
    , connection_state_(ConnectionState::LISTENING)
{
    data_handler_ = std::make_shared<DataHandler>();
    plc_comm_ = std::make_shared<PLCCommunicator>();
    sequence_controller_ = std::make_shared<SequenceController>(data_handler_, nh_);

    // ROS 파라미터에서 설정값 읽기
    loadParameters();
}

IOController::~IOController()
{
    stop();
    LOG_INFO("IOController destroyed");
}

void IOController::initialize()
{
    // IO 디바이스 초기화
    if (!initializeIODevice()) {
        LOG_ERROR("Failed to initialize IO device");
        return;
    }
    // ROS 토픽 설정
    setupROSInterface();

    initializeSequence();

    sequence_controller_->startSequence("equipment_send");
    last_sequnce_name_ = "equipment_send";

    sequence_controller_->startSequence("pin_down");
    last_sequnce_name_ = "pin_down";

    sequence_controller_->startSequence("reset");
    last_sequnce_name_ = "reset";
    
    
    LOG_INFO("IOController initialization completed");
}

bool IOController::initializeIODevice()
{
    return startServerWithRetry(server_bind_ip_, server_port_);
}

bool IOController::startServerWithRetry(const std::string& bind_ip, int port)
{
    current_retry_count_ = 0;

    while (current_retry_count_ < max_retry_count_) {
        LOG_INFO("Starting PLC server on %s:%d... (attempt %d/%d)", bind_ip.c_str(), port, current_retry_count_ + 1, max_retry_count_);

        try {
            if (plc_comm_->startServer(bind_ip, port)) {
                connection_state_ = ConnectionState::LISTENING;  // 새로운 상태
                current_retry_count_ = 0;  // 성공 시 카운터 리셋
                LOG_INFO("PLC server started successfully on %s:%d", bind_ip.c_str(), port);
                return true;
            }
        }
        catch (const std::exception& e) {
            LOG_ERROR("PLC server start exception: %s", e.what());
        }

        current_retry_count_++;
        connection_state_ = ConnectionState::FAILED;

        if (current_retry_count_ < max_retry_count_) {
            LOG_WARNING(
                "PLC server start failed, retrying in %.1f seconds... (%d/%d)", retry_delay_seconds_, current_retry_count_,
                max_retry_count_);
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(retry_delay_seconds_ * 1000)));
        }
    }

    connection_state_ = ConnectionState::FAILED;
    LOG_ERROR("PLC server start failed on %s:%d after %d attempts", bind_ip.c_str(), port, max_retry_count_);
    return false;
}

bool IOController::handleServerRestart()
{
    static auto last_restart_attempt = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    auto time_since_last_attempt = std::chrono::duration<double>(now - last_restart_attempt).count();

    // 설정된 간격마다 서버 재시작 시도
    if (time_since_last_attempt >= reconnect_interval_seconds_) {
        last_restart_attempt = now;
        LOG_INFO("Attempting to restart PLC server on %s:%d...", server_bind_ip_.c_str(), server_port_);

        if (startServerWithRetry(server_bind_ip_, server_port_)) {
            return true;
        }
    }

    return false;
}

static const std::unordered_map<std::string, std::string> kSequenceCommands = {
    {"charge_start", "charge_start"},
    {"charge_stop",  "charge_stop"},
    {"charge_stop_bms",  "charge_stop"},
    {"pin_init", "pin_init"},
    {"pin_up", "pin_up"},
    {"pin_up_3s","pin_up_3s"},
    {"pin_down", "pin_down"},
    {"brake_release_on", "brake_release_on"},
    {"brake_release_off", "brake_release_off"},
    {"quick_stop_on", "quick_stop_on"},
    {"quick_stop_off", "quick_stop_off"},
    {"manual_mode_on", "manual_mode_on"},
    {"manual_mode_off", "manual_mode_off"},
    {"sto_clear_on", "sto_clear_on"},
    {"sto_clear_off", "sto_clear_off"},
    {"equipment_send", "equipment_send"},
    {"bumper_clear_off", "bumper_clear_off"},
    {"bumper_clear_on", "bumper_clear_on"},
    {"reset_false", "reset_false"},
    {"reset", "reset"},
    {"ossd_bypass_on", "ossd_bypass_on"},
    {"ossd_bypass_off", "ossd_bypass_off"},
    {"lidar_ossd_enable", "lidar_ossd_enable"},
    {"lidar_ossd_disable", "lidar_ossd_disable"},
    {"plc_info_send", "plc_info_send"} 
};


/// @note 
void IOController::initializeSequence()
{
    // state 폴더 안에 있음 state별로 분리 해야함

    RegisterPinSequence();
    RegisterChargingSequence();
    RegisterSafetySequence();
    RegisterControllerSequence();
}

void IOController::run()
{
    if (running_) {
        LOG_WARNING("IOController is already running");
        return;
    }

    running_ = true;
    worker_thread_ = std::thread(&IOController::workerLoop, this);

    LOG_INFO("IOController started with %dms communication interval", communication_interval_ms_);
}

void IOController::stop()
{
    if (running_) {
        running_ = false;
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
        LOG_INFO("IOController stopped");
    }
}

void IOController::workerLoop() // 메인 루프
{
    int consecutive_failures = 0;
 
    while (running_ && ros::ok()) {
        try {
            // 서버 상태 확인
            if (!plc_comm_->isServerRunning()) {
                // 서버가 정지된 상태면 재시작 시도
                if (!handleServerRestart()) {
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    continue;
                }
            }
            // LOG_INFO("PLC server is running, checking client connection...");
            // 클라이언트 연결 상태 확인 및 통신
            if (plc_comm_->isClientConnected()) {
                connection_state_ = ConnectionState::CONNECTED;
                // LOG_INFO("PLC client is connected, proceeding with IO communication...");

            
                // IO 데이터 통신
                if (performIOCommunication()) {
                    consecutive_failures = 0;  // 성공 시 실패 카운터 리셋
                    

                    // 핀 에러 + 브레이크 피드백 + PLC info를 200ms마다 묶어서 실행
                    auto now = std::chrono::steady_clock::now();
                    if (last_plc_info_pub_.time_since_epoch().count() == 0 ||
                        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_plc_info_pub_).count() >= 200) {
                        
                        // 핀 에러 스캔
                        scanAndPublishPinErrorsOnce_();

                        // 브레이크 피드백 퍼블리시
                        if (data_handler_->getSafetyBit(SAFETY::FRONT_BUMPPER) || data_handler_->getSafetyBit(SAFETY::REAR_BUMPER)) {
                            // LOG_INFO("front bumper %d, rear bumper %d",
                            //         data_handler_->getSafetyBit(SAFETY::FRONT_BUMPPER),
                            //         data_handler_->getSafetyBit(SAFETY::REAR_BUMPER));
                        }
                        std_msgs::Bool brake_msg;
                        brake_msg.data = !data_handler_->getSafetyBit(SAFETY::BRAKE_RELEASE);
                        brake_pub_.publish(brake_msg);

                        // PLC 정보 퍼블리시
                        publishPlcInfoOnce_();
                        // 에러 체크
                        checkAndPublishDelayedErrors();

                        // Undocking 체크
                        processUndockUpCheck_();

                        // 타임스탬프 갱신
                        last_plc_info_pub_ = now;
                    }
                                                
                                                    // 충전 만 던짐 > 충전 중인데도 미션이 넣어짐(알아서 충전을 끊어짐)
                                                  // 충전 만 던짐 > 충전 중인데도 미션이 넣어짐(알아서 충전을 끊어짐)
                                                  // 충전기랑 주고받는 신호가 필요함
                                                  // 정상적인 사이클 1. 도킹 > amr은 충전릴레이를 on한다(충전시작) > 충전완료 또는 중지 사이클 80% 1분만큼 지속되면 종료, 관제에서 가라는 경우 충전릴레이 off
                                                  // can2 라인을 통해 0xA0(충전기), 데이터 [0]BYTE:1 (충전 요청), 0(충전종료 요청) > 전송주기(2회이상/초당), 충전중일 때도 계속 보내야 함, 종료할 때 0번 날려줘야 함
                                                  // 충전기 > AMR로 알람코드를 만들어서 0xA1 [0]BYTE: 0이면 에러없음, 1부터 에러 있음(ERROR CODE 0~255)  전송주기: 4회/초당
                                                  // OFF할 때는 CAN을 OFF 먼저하고 5초 뒤에 릴레이 꺼라
                    if (sequence_controller_) {
                        sequence_controller_->process(); // 시퀀스 컨트롤러 메인 처리 함수 호출

                        // last_sequnce_name_이 비어있지 않으면 상태 확인
                        if (!last_sequnce_name_.empty()) {
                            current_sequnce_state_ = sequence_controller_->getSequenceState(last_sequnce_name_);
                        
                            if (current_sequnce_state_ == SequenceState::ABORTED && isStateChanged(current_sequnce_state_, prev_sequnce_state_)) {
                                std::string abort_reason = sequence_controller_->getAbortReason();
                                LOG_ERROR("Sequence '%s' aborted: %s", last_sequnce_name_.c_str(), abort_reason.c_str());
                                // s_error_.data = core_msgs::NaviAlarm::ERROR_SEQUENCE_TIMEOUT;
                                // error_pub_.publish(s_error_);

                                prev_sequnce_state_ = current_sequnce_state_;

                                // 시퀀스 리셋 해야 하나?????
                            }
                            else if(current_sequnce_state_ == SequenceState::COMPLETED && isStateChanged(current_sequnce_state_, prev_sequnce_state_)){
                                // 시퀀스 완료 토픽 발행 필요
                                std_msgs::String seq_complete_msg;
                                sequence_complete_pub_.publish(seq_complete_msg); //내용은 안담고 토픽만 발행

                                prev_sequnce_state_ = current_sequnce_state_;
                            }
                        }
                    }
                }
                else {
                    consecutive_failures++;
                    LOG_WARNING("IO communication failed (consecutive failures: %d)", consecutive_failures);

                    // 연속 실패가 임계값 초과시 연결 상태 변경
                    if (consecutive_failures >= max_consecutive_failures_) {
                        LOG_ERROR("Too many consecutive failures (%d), client may be disconnected", max_consecutive_failures_);
                        connection_state_ = ConnectionState::LISTENING;
                        consecutive_failures = 0;
                        s_error_.data = core_msgs::NaviAlarm::ERROR_PLC_IO_COMMUNICATION_FAILED;
                        error_pub_.publish(s_error_);
                    }
                }
            }
            else {
                // 클라이언트가 연결되지 않은 상태
                connection_state_ = ConnectionState::LISTENING;
                consecutive_failures = 0;

                // 주기적으로 연결 대기 상태 로그
                static auto last_log_time = std::chrono::steady_clock::now();
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count();

                if (elapsed >= 10) {  // 10초마다
                    LOG_INFO("Waiting for PLC client connection...");
                    last_log_time = now;
                    s_error_.data = core_msgs::NaviAlarm::ERROR_PLC_NOT_CONNECTED;
                    error_pub_.publish(s_error_);
                }
            }            
        }
        catch (const std::exception& e) {
            LOG_ERROR("Exception in worker loop: %s", e.what());
            connection_state_ = ConnectionState::FAILED;

        }

        // 고정된 100ms 주기로 대기
        std::this_thread::sleep_for(std::chrono::milliseconds(communication_interval_ms_));
    }
}

bool IOController::isStateChanged(SequenceState current, SequenceState previous)
{
    return current != previous;
}
bool IOController::performIOCommunication()
{
    try {
        std::array<uint8_t, IO_DATA_SIZE> input;
        auto output = data_handler_->getOutputArray();

        if (plc_comm_->sendAndReceive(output, input)) {
            data_handler_->setInputArray(input);
            return true;
        }
        else {
            LOG_WARNING("PLC communication failed: %s", plc_comm_->getLastError().c_str());
            return false;
        }
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception during IO communication: %s", e.what());
        return false;
    }
}

inline std::string trim(std::string s) {
    auto not_space = [](int ch){ return !std::isspace(ch); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
    s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
    return s;
}

void IOController::processUndockUpCheck_()
{
    if (!undock_up_check_pending_) return;

    auto now_tp = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now_tp - undock_up_check_start_tp_).count();
    if (elapsed < undock_up_check_delay_sec_) return;

    const bool front_up = data_handler_->getPinBit(PIN::FRONT_UP_PX);
    const bool rear_up  = data_handler_->getPinBit(PIN::REAR_UP_PX);

    if (b_last_pin_up_command_) {  // <--- 핀업 명령 있었을 때만 체크
        if ((!front_up || !rear_up) && fabs(f_linear_x_)> 0.05) {
            s_error_.data = core_msgs::NaviAlarm::ERROR_PIN_NOT_ENGAGED;
            error_pub_.publish(s_error_);
            LOG_WARNING("[UNDOCK] UP PX not detected after %.1f sec (F:%d, R:%d) -> publish alarm",
                        undock_up_check_delay_sec_, front_up, rear_up);
        } else {
            LOG_INFO("[UNDOCK] UP PX OK after %.1f sec (F:%d, R:%d)",
                     undock_up_check_delay_sec_, front_up, rear_up);
        }
    } else {
        LOG_INFO("[UNDOCK] Skip UP PX check (no prior pin-up command)");
    }

    undock_up_check_pending_ = false; // 1회 검사 후 종료
}

void IOController::parseAndExecuteCommand(const std::string& raw_command)
{

    // --- 시퀀스 처리(우선) : 데이터 기반 ---
    std::string command = trim(raw_command);

    if (auto it = kSequenceCommands.find(command); it != kSequenceCommands.end()) {
        const std::string& seq_name = it->second;
        if (command == "charge_stop") {

            bool relay_on = data_handler_->getSensorBit(SENSOR::REAR_CHARGE_RELAY_CHECK_SIGNAL);

            if(relay_on)
            {
                LOG_INFO("Delay 3s before charge_stop sequence");
                std::this_thread::sleep_for(std::chrono::seconds(3)); 
            }
            else
            {
                LOG_INFO("Already charge relay off");
                std_msgs::Bool charge_state_msg;
                charge_state_msg.data = false;              
                charge_state_pub_.publish(charge_state_msg);
                return;
            }
        
            //슬램도킹이던, 파렛트 도킹이던, 충전도킹이던 도킹아웃할 때 구분안하고 chargestop을 줌. 릴레이 내려가 있으면 무시하고, charge stop done을 달라. 릴레이 체크해서 괜찮으면 그냥 완료주고 안괜찮으면 끊고 끊은것까지 확인해서 주고.
        }
        sequence_controller_->SetOutputCommand(command);
        if (sequence_controller_->startSequence(seq_name)) {
            last_sequnce_name_ = seq_name;
            LOG_INFO("Started sequence: %s", seq_name.c_str());
        } else {
            LOG_WARNING("Failed to start sequence: %s", seq_name.c_str());
        }
        return; // 시퀀스면 바로 종료
    }
    

    std::istringstream iss(command);
    std::string token;

    while (std::getline(iss, token, ',')) {
        try {
            size_t eq_pos = token.find('=');
            if (eq_pos != std::string::npos) {
                std::string key = token.substr(0, eq_pos);
                int value = std::stoi(token.substr(eq_pos + 1));

                // 명령 매핑
                if (key == "LIFT") {
                    data_handler_->setOutputValue(OUTPUT::LIFT, value);
                }
                else if (key == "LIFT_TURN") {
                    data_handler_->setOutputValue(OUTPUT::LIFT_TURN, value);
                }
                else if (key == "CONVEYOR") {
                    data_handler_->setOutputValue(OUTPUT::CONVEYOR, value);
                }
                else if (key == "UPPER") {
                    data_handler_->setOutputValue(OUTPUT::UPPER, value);
                }
                else if (key == "COMMAND") {
                    data_handler_->setOutputValue(OUTPUT::COMMAND, value);
                }
                else {
                    LOG_WARNING("Unknown command key: %s", key.c_str());
                    continue;
                }
                LOG_INFO("Set %s = %d", key.c_str(), value);
            }
        }
        catch (const std::exception& e) {
            LOG_ERROR("Error parsing command token '%s': %s", token.c_str(), e.what());
        }
    }
}

// 상태 조회 메서드들
IOController::ConnectionState IOController::getConnectionState() const
{
    return connection_state_;
}

int IOController::getCurrentRetryCount() const
{
    return current_retry_count_;
}

std::string IOController::getConnectionStateString() const
{
    switch (connection_state_) {
        case ConnectionState::CONNECTED:
            return "CONNECTED";
        case ConnectionState::LISTENING:
            return "LISTENING";  // 새로운 상태
        case ConnectionState::RECONNECTING:
            return "RECONNECTING";
        case ConnectionState::FAILED:
            return "FAILED";
        default:
            return "UNKNOWN";
    }
}

// 설정값 조회 메서드들
std::string IOController::getServerBindIP() const
{
    return server_bind_ip_;
}

int IOController::getServerPort() const
{
    return server_port_;
}

std::string IOController::getServerState() const
{
    if (plc_comm_) {
        return plc_comm_->getServerStateString();
    }
    return "UNKNOWN";
}

std::string IOController::getConnectedClientIP() const
{
    if (plc_comm_) {
        return plc_comm_->getConnectedClientIP();
    }
    return "";
}


}  // namespace NaviFra