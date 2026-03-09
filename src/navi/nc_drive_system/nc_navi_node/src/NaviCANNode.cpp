#include "nc_navi_node/NaviCANNode.hpp"

using namespace NaviFra::NaviCAN::Node;
using namespace std::chrono_literals;

bool NaviCANNode::initialize()
{

    try {
        navican_ = std::make_unique<NaviCANCore>();
        if (navican_->initialize() == false) {
            navican_.reset();
            return false;

        }
    } catch(std::exception& e) {
        return false;
    } 

    // 모든 모터가 enabled될 때까지 대기
    if (WaitForMotorsEnabled() == false) {
        navican_->finalize();
        return false;
    }

    // ROS publishers, subscribers, and services 초기화
    pub_msgs_ = nh_.advertise<motor_msgs::NavicanDataArray>("navican/node/msgs", ROS_QUEUE_SIZE);
    sub_target_ = nh_.subscribe("navican/node/target", ROS_QUEUE_SIZE, &NaviCANNode::SubscribeTarget, this);
    sub_encoder_zero_ = nh_.subscribe("navican/node/encoder_zero", ROS_QUEUE_SIZE, &NaviCANNode::SubscribeEncoderZero, this);
    srv_cmd_ = nh_.advertiseService("navican/node/srv", &NaviCANNode::ServiceCmd, this);
    
    // 콜백 등록
    navican_->OnSyncWithMaster([this](uint8_t cnt, const lely::io::TimerBase::time_point&) {
        PublishMsgs();
    });

    NLOG(info) << "NaviCANNode initialized.";

    return true;
}

void NaviCANNode::SubscribeTarget(const motor_msgs::MotorTargetInfo::ConstPtr& msg)
{
    // 중복 실행 방지
    if (is_updating_target_.exchange(true)) return;
    Guard guard{is_updating_target_};

    if (!navican_) {
        return;
    }

    for (const auto& info : msg->data) {
        navican_->setTarget(info.motor_id, info.target);
    }
}

void NaviCANNode::SubscribeEncoderZero(const std_msgs::String::ConstPtr& msg)
{
    // 중복 실행 방지
    if (is_updating_encoder_.exchange(true)) return;
    Guard guard{is_updating_encoder_};

    try {
        navican_->presetEncoderAll(); 
        NLOG(info) << " reset successfully.";
    } catch (const std::exception& e) {
        NLOG(info) << "Error resetting encoder : "  << e.what();
    }
}

bool NaviCANNode::finalize(void)
{

    if(navican_.get()) {
        navican_->OnSyncWithMaster(nullptr);
        navican_->finalize();
        navican_.reset();
    }
    return true;
}


void NaviCANNode::PublishMsgs(void) 
{
    // 중복 실행 방지
    if (is_updating_msgs_.exchange(true)) return;
    Guard guard{is_updating_msgs_};

    msgs_.motors.clear();

    int bus_state = navican_->getCanBusState();
    auto controllers = navican_->getControllers();

    msgs_.motors.reserve(controllers.size());

    for(auto& [motor_id, controller] : controllers) {
        status_word_stream_.str("");
        status_word_stream_.clear();
        status_word_stream_ << "0x" << std::uppercase << std::hex 
                      << std::setfill('0') << std::setw(4) 
                      << controller->getStatus();
        msgs_.motors.emplace_back();
        auto& data = msgs_.motors.back();
        
        data.motor_id = motor_id;

        // MotorDriver: State
        data.motor_state_text = controller->getStateText();
        data.is_enable = controller->isEnable();
        data.is_fault = controller->isFault();
        data.status_word = controller->getStatus();
        data.status_word_text = status_word_stream_.str();
    
        // MotorDriver : Target
        data.operation_mode = controller->getOperationMode();
        data.target = controller->getTarget();
        data.actual_velocity = controller->getSpeed();
        data.actual_position = controller->getPosition();

        // MotorExtraInfo
        data.voltage = controller->getVoltage();
        data.current = controller->getCurrent();
        data.error_code = controller->getErrorCode();
        data.error_text = controller->getErrorMessage();

        data.external_encoder = controller->getPosition();
        data.external_encoder_diff = data.external_encoder - prev_external_encoders_[motor_id];
        data.bus_state = bus_state;
        data.timestamp = navican_->getLatestRpdoWriteTimeCount(motor_id);

        prev_external_encoders_[motor_id] = data.external_encoder;
    }
    pub_msgs_.publish(msgs_);
}

bool NaviCANNode::ServiceCmd(motor_msgs::MotorCmdRequest& req, motor_msgs::MotorCmdResponse& res)
{

    // 중복 실행 방지
    if (is_processing_cmd_.exchange(true)) return false;
    Guard guard{is_processing_cmd_};

    NLOG(info) << "Recv Motor CMD : " << req.s_cmd;
    
    if (!navican_) {
        NLOG(info) << "NaviCANCore not initialized.";
        return false;
    }

    try {
        if (req.s_cmd == "resetError") {
            navican_->resetErrorAll();
            res.b_is_response = true;
            return true;
        }
        else if (req.s_cmd == "enable") {
            if (req.motor_id == 999) {
                navican_->enableAll();
            }
            else {
                navican_->enable(req.motor_id);
            }
            res.b_is_response = true;
            return true;
        }
        else if (req.s_cmd == "disable") {
            if (req.motor_id == 999) {
                navican_->disableAll();
            } else {
                navican_->disable(req.motor_id);
            }
            res.b_is_response = true;
            return true;
        }
        else if (req.s_cmd == "shutdown") {
            if (req.motor_id == 999) {
                navican_->shutdownAll();
            } else {
                navican_->shutdown(req.motor_id);
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


bool NaviCANNode::WaitForMotorsEnabled(void)
{
    using namespace NaviFra::NaviCAN::Node;
    
    NLOG(info) << "Waiting for motors to become enabled...";
    
    const auto controllers = navican_->getControllers();
    const auto timeout = std::chrono::steady_clock::now() + std::chrono::seconds(TIMEOUT_WAITING_FOR_NODES_ENABLED);
    
    // 람다 : 모든 노드가 활성화되었는지 확인
    auto all_enabled = [&]() {
        return std::all_of(controllers.begin(), controllers.end(),
            [](const auto& pair) { return pair.second->isEnable(); });
    };
    
    // 확인 반복, 타임아웃 까지
    while (std::chrono::steady_clock::now() < timeout) {
        try {
            // 1초 대기
            std::this_thread::sleep_for(1s);
            NLOG(info) << "Checking if all motors are enabled...";
            // 모든 모터가 enabled 가 아니면 재확인
            if (all_enabled() == false) continue;

            // 모든 모터가 enabled
            NLOG(info) << "All required motors are OPERATIONAL";
            return true;
        } catch (...) {
            return false;
        }
    }
    
    // 로그출력 (타임아웃)
    auto final_count = std::count_if(controllers.begin(), controllers.end(),
        [](const auto& pair) { return pair.second->isEnable(); });
    NLOG(info) << "Timeout: Only " << final_count << " motors are ENABLED";
    return false;
}
