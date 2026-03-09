#include "NaviCANBridge.hpp"

#include <signal.h>

using namespace NaviFra;
using namespace std::chrono_literals;

bool NaviCANBridge::initialize()
{
    try {
        navi_can_ = std::make_shared<NaviCANCore>();
        if (!navi_can_->initialize()) {
            navi_can_.reset();
            throw std::runtime_error("Failed to initialize NaviCANCore");
        }
    }
    catch (std::exception& e) {
        throw;
    }

    // 모든 노드가 enabled될 때까지 대기
    if (WaitForNodesEnabled() == false) {
        navi_can_->finalize();
        return false;
    }

    // ROS publishers, subscribers, and services 초기화
    pub_motor_info_ = nh_.advertise<motor_msgs::MotorDataInfo>("motor_data/info", 5);
    sub_motor_target_ = nh_.subscribe("motor_target", 5, &NaviCANBridge::SubscribeCmdvel, this);
    sub_enc_zero_ = nh_.subscribe("encoder_zero", 5, &NaviCANBridge::RecvEncoderZero, this);
    srv_motor_cmd_ = nh_.advertiseService("motor_cmd", &NaviCANBridge::SubscribeMotorCmd, this);

    // 콜백 등록
    navi_can_->OnSyncWithMaster([this](uint8_t cnt, const lely::io::TimerBase::time_point&) { ReceivedMotorInfo(); });

    NLOG(info) << "NaviCANCore initialized.";

    return true;
}

void NaviCANBridge::finalize()
{
    NLOG(info) << "NaviCANBridge finalizing...";

    // NaviCANCore 종료
    if (navi_can_.get()) {
        navi_can_->OnSyncWithMaster(nullptr);
        navi_can_->finalize();
        navi_can_.reset();
    }

    NLOG(info) << "NaviCANBridge finalized.";
}

void NaviCANBridge::ReceivedMotorInfo()
{
    // 중복 실행 방지
    if (!motor_info_mutex_.try_lock())
        return;
    std::lock_guard<std::mutex> lock(motor_info_mutex_, std::adopt_lock);

    // 기존 데이터 클리어
    motor_data_info_.data.clear();

    int bus_state = navi_can_->getCanBusState();
    auto controllers = navi_can_->getControllers();

    // 메모리 사전 할당
    motor_data_info_.data.reserve(controllers.size());

    for (auto& [motor_id, controller] : controllers) {
        // 16진수 상태 문자열 생성 (stringstream 재사용)
        status_stream_.str("");
        status_stream_.clear();
        status_stream_ << "0x" << std::uppercase << std::hex << std::setfill('0') << std::setw(4) << controller->getStatus();

        // emplace_back으로 복사 비용 방지
        motor_data_info_.data.emplace_back();
        auto& data = motor_data_info_.data.back();

        // 모터 데이터 설정
        data.motor_id = motor_id;
        data.status = controller->getStatus();
        data.x_status = status_stream_.str();
        data.s_status = controller->getStateText();
        data.is_enable = controller->isEnable();
        data.is_error = controller->isFault();
        data.sto_code = controller->getSTOCode();
        data.error_code = controller->getErrorCode();
        data.error_msg = controller->getErrorMessage();
        data.current = controller->getCurrent();
        data.voltage = controller->getVoltage();
        data.input_velocity = controller->getTarget();
        data.feedback_velocity = controller->getSpeed();
        data.input_angle = data.feedback_angle = 0;  // 미사용
        data.encoder = controller->getPosition();
        data.acc_encoder = data.encoder - prev_encoder_[motor_id];
        data.bus_state = bus_state;
        data.last_update_time = navi_can_->getLatestRpdoWriteTimeCount(motor_id);
        prev_encoder_[motor_id] = data.encoder;
    }

    pub_motor_info_.publish(motor_data_info_);
}

void NaviCANBridge::RecvEncoderZero(const std_msgs::String::ConstPtr& msg)
{
    try {
        navi_can_->presetEncoderAll();
        NLOG(info) << " reset successfully.";
    }
    catch (const std::exception& e) {
        NLOG(info) << "Error resetting encoder : " << e.what();
    }
}

bool NaviCANBridge::SubscribeMotorCmd(motor_msgs::MotorCmdRequest& req, motor_msgs::MotorCmdResponse& res)
{
    // 중복 실행 방지
    if (!motor_cmd_mutex_.try_lock())
        return false;
    std::lock_guard<std::mutex> lock(motor_cmd_mutex_, std::adopt_lock);

    NLOG(info) << "Recv Motor CMD : " << req.s_cmd;

    if (!navi_can_) {
        NLOG(info) << "NaviCANCore not initialized.";
        return false;
    }

    try {
        if (req.s_cmd == "resetError") {
            navi_can_->resetErrorAll();
            res.b_is_response = true;
            return true;
        }
        else if (req.s_cmd == "homing") {
            res.b_is_response = navi_can_->homing(req.motor_id, req.homing_method, std::chrono::milliseconds(req.homing_timeout_ms));
            return true;
        }
        else if (req.s_cmd == "enable") {
            if (req.motor_id == 999) {
                navi_can_->enableAll();
            }
            else {
                navi_can_->enable(req.motor_id);
            }
            res.b_is_response = true;
            return true;
        }
        else if (req.s_cmd == "disable") {
            if (req.motor_id == 999) {
                navi_can_->disableAll();
            }
            else {
                navi_can_->disable(req.motor_id);
            }
            res.b_is_response = true;
            return true;
        }
        else if (req.s_cmd == "shutdown") {
            if (req.motor_id == 999) {
                navi_can_->shutdownAll();
            }
            else {
                navi_can_->shutdown(req.motor_id);
            }
            res.b_is_response = true;
            return true;
        }
    }
    catch (const std::exception& e) {
        NLOG(info) << e.what();
        return false;
    }

    return false;
}

void NaviCANBridge::SubscribeCmdvel(const motor_msgs::MotorTargetInfo::ConstPtr& msg)
{
    // 중복 실행 방지
    if (!cmdvel_mutex_.try_lock())
        return;
    std::lock_guard<std::mutex> lock(cmdvel_mutex_, std::adopt_lock);

    if (!navi_can_) {
        return;
    }

    for (const auto& info : msg->data) {
        navi_can_->setTarget(info.motor_id, info.target);
    }
}

bool NaviCANBridge::WaitForNodesEnabled(void)
{
    using namespace NaviFra::NaviCAN::Bridge;

    NLOG(info) << "Waiting for nodes to become enabled...";

    const auto controllers = navi_can_->getControllers();
    const auto timeout = std::chrono::steady_clock::now() + std::chrono::seconds(TIMEOUT_WAITING_FOR_NODES_ENABLED);

    // 람다 : 모든 노드가 활성화되었는지 확인
    auto all_enabled = [&]() {
        return std::all_of(controllers.begin(), controllers.end(), [](const auto& pair) { return pair.second->isEnable(); });
    };

    // 확인 반복, 타임아웃 까지
    while (std::chrono::steady_clock::now() < timeout) {
        try {
            // 1초 대기
            std::this_thread::sleep_for(1s);
            NLOG(info) << "Checking if all nodes are enabled...";
            // 모든 노드가 enabled 가 아니면 재확인
            if (all_enabled() == false)
                continue;

            // 모든 노드가 enabled
            NLOG(info) << "All required nodes are OPERATIONAL";
            return true;
        }
        catch (...) {
            return false;
        }
    }

    // 로그출력 (타임아웃)
    auto final_count = std::count_if(controllers.begin(), controllers.end(), [](const auto& pair) { return pair.second->isEnable(); });
    NLOG(info) << "Timeout: Only " << final_count << " nodes are ENABLED";
    return false;
}

int main(int argc, char** argv)
{
    try {
        ros::init(argc, argv, "nc_navi_bridge");

        ros::NodeHandle nh;
        ros::NodeHandle nhp("~");
        NaviCANBridge bridge(nh, nhp);
        if (!bridge.initialize()) {
            NLOG(info) << "Failed to initialize NaviCANBridge.";
            return -1;
        }

        ros::spin();

        bridge.finalize();
        return 0;
    }
    catch (const std::exception& e) {
        NLOG(severity_level::error) << "Fatal error: " << e.what();
        return -1;
    }
    catch (...) {
        NLOG(severity_level::error) << "Unknown fatal error occurred";
        return -1;
    }

    return 0;
}