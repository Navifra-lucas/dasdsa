#include "nc_driver_bridge.hpp"

#include "core/util/logger.hpp"

Driver::Driver(ros::NodeHandle& nh, ros::NodeHandle& nhp)
    : nh_(nh)
    , nhp_(nhp)
{
    LOG_INFO("Driver Create");
    // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    Initialize(false);
    b_param_init_ = true;
}

Driver::~Driver()
{
    Shutdown();
}

void Driver::Shutdown()
{
    LOG_INFO("thread wait");
    int n_size = vec_communicator_.size();

    for (int i = 0; i < n_size; i++) {
        vec_communicator_[i]->Stop();
    }
    for (int i = 0; i < n_size; i++) {
        delete vec_communicator_[i];
    }
    th_heartbeat_.join();

    
    // for (auto* communicator : vec_communicator_) {
    //     communicator->Terminate();  // 스레드 종료
    //     delete communicator;
    // }
    vec_communicator_.clear();  // 벡터 정리
    LOG_INFO("destructor");
}

void Driver::Initialize(bool b_only_param_update)
{
    LOG_INFO("Update Param start %d", b_only_param_update);
    o_param_.UpdateParam(nh_, nhp_, st_interface_param_, st_driver_param_);
    LOG_INFO("Update Param done");
    // if (b_only_param_update && n_kinematics_type_pre_ != st_driver_param_.n_kinematics_type) {
    //     Shutdown();
    // }
    if (!b_only_param_update)  // update all
    {
        if (st_interface_param_.b_pcan_use) {
            LOG_INFO("nc_custom_driver_main b_pcan_use");
            if (st_interface_param_.st_pcan_param.n_pcan_mode == PCAN_MODE::PCAN_SAMYANG) {
                vec_communicator_.emplace_back(new Pcan_driver_samyang_dd);
            }
            else if (st_interface_param_.st_pcan_param.n_pcan_mode == PCAN_MODE::PCAN_CURTIS) {
                vec_communicator_.emplace_back(new Pcan_driver_curtis_sd);
            }
        }
        if (st_interface_param_.b_can_use) {
            LOG_INFO("nc_custom_driver_main b_can_use");
            if (st_interface_param_.st_can_param.n_can_mode == CAN_MODE::CAN_AMC)
                vec_communicator_.emplace_back(new Can_driver_amc_dd);
            else if (st_interface_param_.st_can_param.n_can_mode == CAN_MODE::CAN_CURTIS) {
                vec_communicator_.emplace_back(new Can_driver_curtis_sd);
            }
        }
        if (st_interface_param_.b_serial_use) {
            LOG_INFO("nc_custom_driver_main b_serial_use");
        }

        for (int i = 0; i < vec_communicator_.size(); i++) {
            vec_communicator_[i]->SetInterfaceParam(st_interface_param_); // interface param 가져오기
            vec_communicator_[i]->SetDriverParam(st_driver_param_); // driver param 가져오기
            vec_communicator_[i]->InterfaceOpen(); // 인터페이스 열기(can/pcan_base 에서 open)
            vec_communicator_[i]->RegisteCallbackFunc( 
                "motor_data_callback", std::bind(&Driver::HandleMotorData, this, std::placeholders::_1)); // 하위단에서 motordata msg 담아 토픽으로 pub 하기위한 콜백.
            vec_communicator_[i]->Initialize(); // 하위단 드라이버 코드 initialize
        }
    }
    else  // only update param
    {
        for (int i = 0; i < vec_communicator_.size(); i++) {
            vec_communicator_[i]->SetInterfaceParam(st_interface_param_);
            vec_communicator_[i]->SetDriverParam(st_driver_param_);
        }
    }
    if (!b_only_param_update) {
        RegistTalker();
        RegistListener();
    }

    LOG_INFO("Regist done");

}  

void Driver::RegistTalker()
{
    // ROS 퍼블리셔 등록
    pub_motor_data_ = nh_.advertise<motor_msgs::MotorDataInfo>("motor_data/info", 10);
    // pub_motor_info_ = nh_.advertise<core_msgs::MotorInfo>("motor_info", 5);
}

void Driver::RegistListener()
{
    // ROS 서브스크라이버 등록
    sub_motor_target_ = nh_.subscribe("motor_target", 10, &Driver::SubscribeMotorTarget, this);
    sub_motor_brake_ = nh_.subscribe("motor_brake", 10, &Driver::SubscribeMotorBrake, this);
    // sub_motor_warning_ = nh_.subscribe("motor_warning", 10, &Driver::SubscribeMotorWarning, this);
    // ROS 서비스 서버 등록
    srv_motor_cmd_ = nh_.advertiseService("motor_cmd", &Driver::MotorCmdCallback, this);
    sub_motor_driver_init_ = nh_.subscribe("motor_driver/init", 5, &Driver::RecvMotorDriverInit, this);
}

void Driver::SubscribeMotorBrake(const motor_msgs::MotorBrake::ConstPtr& msg)
{
    motor_msgs::MotorBrake o_motor_brake_data = *msg;
    if(!o_motor_brake_data.b_brake_control)
        return;
    for (int i = 0; i < vec_communicator_.size(); i++) {
            LOG_INFO("SetBrakeOn : %d", o_motor_brake_data.b_brake_target);
            vec_communicator_[i]->SetBrakeOn(o_motor_brake_data.b_brake_target);
        }
    if(o_motor_brake_data.b_link_brake_and_servo)
        for (int i = 0; i < vec_communicator_.size(); i++) {
            LOG_INFO("SetServoOn : %d", o_motor_brake_data.b_servo_on);
            vec_communicator_[i]->SetServoOn(o_motor_brake_data.b_servo_on);
        }
        
}

void Driver::SubscribeMotorTarget(const motor_msgs::MotorTargetInfo::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mtx_data_);
    // o_motor_target_info_ = *msg;
    for (const auto& motor_target : msg->data) {
        int motor_id = motor_target.motor_id;  
        float target = motor_target.target;
        int operation_mode = motor_target.operation_mode;
        // LOG_INFO("target motor_id: %d, target: %.2f, operation_mode: %d", motor_id, target, operation_mode);
        if (motor_id >= MotorID::FL_TRACTION && motor_id <= MotorID::RR_STEER) {   // 주행 FL(0), FR(1), RL(2), RR(3) 조향 FL(4), FR(5), RL(6), RR(7)
            switch (motor_id) {
                // 주행
                case MotorID::FL_TRACTION:  // FL
                    st_cmd_.f_FL_target_rpm = target;
                    break;
                case MotorID::FR_TRACTION:  // FR
                    st_cmd_.f_FR_target_rpm = target;
                    break;
                case MotorID::RL_TRACTION:  // RL
                    st_cmd_.f_RL_target_rpm = target;
                    break;
                case MotorID::RR_TRACTION:  // RR
                    st_cmd_.f_RR_target_rpm = target;
                    break;
                // 조향
                case MotorID::FL_STEER: 
                    st_cmd_.f_FL_target_deg = target; 
                    break;
                case MotorID::FR_STEER: 
                    st_cmd_.f_FR_target_deg = target; 
                    break;
                case MotorID::RL_STEER: 
                    st_cmd_.f_RL_target_deg = target; 
                    break;
                case MotorID::RR_STEER: 
                    st_cmd_.f_RR_target_deg = target; 
                    break;
                default:
                    LOG_INFO("Unexpected motor_id: %d", motor_id);
                    break;
            }
        } else {
            LOG_INFO("Invalid motor_id: %d", motor_id);
        }
    }
    for (int i = 0; i < vec_communicator_.size(); i++) {
        vec_communicator_[i]->Write(st_cmd_);
    }
}

// void Driver::SubscribeMotorWarning(const std_msgs::String::ConstPtr& msg)
// {
//     std::lock_guard<std::mutex> lock(mtx_data_);
//     o_motor_warning_info_ = *msg;
//     LOG_INFO("Received motor warning: %s", msg->data.c_str());
// }

bool Driver::MotorCmdCallback(motor_msgs::MotorCmd::Request& request, motor_msgs::MotorCmd::Response& response)
{
    LOG_INFO("Received motor command request: motor_id=%d, cmd=%s", request.motor_id, request.s_cmd.c_str());
    // 명령 실행 전: b_is_response = false
    response.b_is_response = false;
    ControlWord control_word;
    
    if (request.s_cmd == "enable") {
        control_word = CONTROL_WORD_ENABLE_OPERATION;
    }
    else if (request.s_cmd == "disable") {
        control_word = CONTROL_WORD_DISABLE_OPERATION;  // 0x06
    }
    else if (request.s_cmd == "resetError") {
        control_word = CONTROL_WORD_FAULT_RESET;  // 0x80
    }
    else {
        LOG_ERROR("Invalid command: %s", request.s_cmd.c_str());
    }

    for (int i = 0; i < vec_communicator_.size(); i++) {
        LOG_INFO("sendControlWord");
        vec_communicator_[i]->sendControlWord(control_word, request.motor_id);
    }
    
    // 응답 설정
    response.b_is_response = true;
    
    return true;
}

void Driver::HandleMotorData(const boost::any& any_type_var) {
    motor_msgs::MotorDataInfo data = boost::any_cast<motor_msgs::MotorDataInfo>(any_type_var);  
    for (auto& motor : data.data) {
        std::stringstream ss;
        ss << "0x" << std::uppercase << std::hex << std::setfill('0') << std::setw(4) << motor.status;  // 16진수로 변환
        std::string str_status = ss.str();
        motor.x_status = str_status;
        motor.s_status = GetStatusText(motor.status);
        motor.is_enable = false;
        if(motor.s_status == "OPERATION_ENABLE")
            motor.is_enable = true;
    }
    pub_motor_data_.publish(data);
}

void Driver::RecvMotorDriverInit(const std_msgs::String::ConstPtr& msg)
{
    string str = msg->data;
    for (int i = 0; i < vec_communicator_.size(); i++) {
        vec_communicator_[i]->MotorDriverInit(str);
    }
}

string Driver::GetStatusText(int n_state)
{
    string str_status = "";

    static const uint16_t r = (1 << StatusWord::SW_READY_TO_SWITCH_ON);
    static const uint16_t s = (1 << StatusWord::SW_SWITCHED_ON);
    static const uint16_t o = (1 << StatusWord::SW_OPERATION_ENABLED);
    static const uint16_t f = (1 << StatusWord::SW_FAULT);
    static const uint16_t q = (1 << StatusWord::SW_QUICK_STOP);
    static const uint16_t d = (1 << StatusWord::SW_SWITCH_ON_DISABLED);

    uint16_t state = n_state & (d | q | f | o | s | r);
    switch (state) {
        case (0 | 0 | 0 | 0 | 0 | 0):
        case (0 | q | 0 | 0 | 0 | 0):
            str_status = "NOT_READY_TO_SWITCH_ON";
            break;

        case (d | 0 | 0 | 0 | 0 | 0):
        case (d | q | 0 | 0 | 0 | 0):
            str_status = "SWITCH_ON_DISABLED";
            break;

        case (0 | q | 0 | 0 | 0 | r):
            str_status = "READY_TO_SWITCH_ON";
            break;

        case (0 | q | 0 | 0 | s | r):
            str_status = "SWITCHED_ON";
            break;

        case (0 | q | 0 | o | s | r):
            str_status = "OPERATION_ENABLE";
            break;

        case (0 | 0 | 0 | o | s | r):
            str_status = "QUICK_STOP_ACTIVE";
            break;

        case (0 | 0 | f | o | s | r):
        case (0 | q | f | o | s | r):
            str_status = "FAULT_REACTION_ACTIVE";
            break;

        case (0 | 0 | f | 0 | 0 | 0):
        case (0 | q | f | 0 | 0 | 0):
            str_status = "FAULT";
            break;
        default:
            break;
    }
    return str_status;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nc_custom_driver");

#ifdef NAVIFRA_LICENSE_ON

    License o_license_checker;
    if (o_license_checker.CheckLicence() == 0) {
        LOG_ERROR("************************************************");
        LOG_ERROR("DRIVER Your License Not Resiter. Terminate the process.");
        LOG_ERROR("************************************************");
        return 0;
    }
    else {
        LOG_INFO("DRIVER License is Resitered.");
    }

#else  // license 옵션 안건 경우
    LOG_WARNING("************************************************");
    LOG_WARNING("DRIVER License Not Checked!  Please Check your License.");
    LOG_WARNING("************************************************");
#endif

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    Driver motor_driver(nh, nhp);
    ros::spin();

    return 0;
}