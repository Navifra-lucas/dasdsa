
#include "interface/can/can_driver_curtis_sd.hpp"

#include "core_msgs/MotorServoStatus.h"
#include "core/util/logger.hpp"

using namespace NaviFra;

enum NMTState
{
    NMT_BOOTUP = 0x00,
    NMT_STOPPED = 0x04,
    NMT_OPERATIONAL = 0x05,
    NMT_PRE_OPERATIONAL = 0x7F
};

void Can_driver_curtis_sd::Stop()
{
    LOG_INFO("Stop");

    CAN_CURTIS_SD_WRITE_1 uo_fl_write_1;

    // Interlock on
    uo_fl_write_1.un8_interlock = int32_t(0);  
    // target 0
    uo_fl_write_1.un16_targetRPM = int32_t(0);
    
    Can_driver::Write(
        CanID::CAN_RPDO1_ID + st_interface_param_.st_can_param.n_FL_traction_can_id, uo_fl_write_1.byte_space, 8);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

void Can_driver_curtis_sd::Initialize()
{
    b_init_ = false;
    Can_driver::RegisteCallbackFunc("CanCallback", std::bind(&Can_driver_curtis_sd::CanCallback, this, std::placeholders::_1));
    can_frame Message;
    Message.can_id = 0;
    Message.can_dlc = 2;
    Message.data[0] = NMTCommand::OPERATIONAL;
    Message.data[1] = st_interface_param_.st_can_param.n_FL_abssteer_can_id; // 조향 절댓값 엔코더 받기위한 명령
    Can_driver::Write(Message);
    
    // pdo 맵핑은 윈도우 툴 사용해서 함.
    std::this_thread::sleep_for(std::chrono::milliseconds(10));  // NMT 명령 후 짧은 지연 필요
    
    // controlword 설정 따로 필요없음.
    // MotorSwitchOn(MotorID::FL_TRACTION, 3);
    // MotorSwitchOn(MotorID::FL_STEER, 3);
    
    auto now = std::chrono::system_clock::now();
    tp_fl_traction_ = now;
    // Can_driver::Write(0x080, 0, 0, 0, 0, 0, 0, 0, 0);
    b_init_ = true;
}

void Can_driver_curtis_sd::ControlOff(bool data)
{
   // b_control_off_ = data;
   if (data) {
    LOG_INFO("SET ControlOff ON");
    }
    if (!data) {
        LOG_INFO("SET ControlOff OFF");
    }
}

void Can_driver_curtis_sd::ReInitializeCheck()
{}

void Can_driver_curtis_sd::MotorDriverInit(string& str_data)
{
    // Activate emergency shutdown byte0 = 0x23 Index : 0x200C, sub : 0x00
    LOG_INFO("Motor Reinit Start");
    b_init_ = false;

    Can_driver::uninit();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));  // NMT 명령 후 짧은 지연 필요

    can_frame Message;
    Message.can_id = 0;
    Message.can_dlc = 2;
    Message.data[0] = NMTCommand::OPERATIONAL;
    Message.data[1] = st_interface_param_.st_can_param.n_FL_abssteer_can_id; // 조향 절댓값 엔코더 받기위한 명령
    try{
        Can_driver::Write(Message);
        nmt_response_manager_.waitForResponse(
            CanID::CAN_NMT_STATUS_ID + st_interface_param_.st_can_param.n_FL_abssteer_can_id, 10000);
    }
    catch(const std::exception& ex) {
        LOG_ERROR("%s", ex.what());
    }

    // pdo 맵핑은 윈도우 툴 사용해서 함.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));  // NMT 명령 후 짧은 지연 필요

    // controlword 설정 따로 필요없음.
    // MotorSwitchOn(MotorID::FL_TRACTION, 3);
    // MotorSwitchOn(MotorID::FL_STEER, 3);>
    LOG_INFO("Motor Reinit Finish");

    auto now = std::chrono::system_clock::now();
    tp_fl_traction_ = now;
    // Can_driver::Write(0x080, 0, 0, 0, 0, 0, 0, 0, 0);
    b_init_ = true;
}

void Can_driver_curtis_sd::MotorErrorReset(std::string& str_data)
{
    LOG_INFO("Motor Reset Start");
    // MotorSwitchOn(MotorID::FL_TRACTION, 3);
    // MotorSwitchOn(MotorID::FL_STEER, 3);
    LOG_INFO("Motor Reset Finish");
}

void Can_driver_curtis_sd::EncoderZero(string& str_data)
{
    LOG_INFO("SetEncoderZero");
    Can_driver::Write(
        CanID::CAN_SDO_REQUEST_ID + (uint8_t)st_interface_param_.st_can_param.n_FL_abssteer_can_id, 0x22, 0x03, 0x60, 0x00, 0x00, 0x00, 0x00,
        0x00);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    Can_driver::Write(
        CanID::CAN_SDO_REQUEST_ID + (uint8_t)st_interface_param_.st_can_param.n_FL_abssteer_can_id, 0x22, 0x10, 0x10, 0x01, 0x73, 0x61, 0x76,
        0x65);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void Can_driver_curtis_sd::Write(const Wheel_Cmd_t& st_cmd)
{
    if (!b_init_)
        return;

    CAN_CURTIS_SD_WRITE_1 uo_write1;

    // 주행 명령
    int n_target_rpm_f = st_cmd.f_FL_target_rpm;
    float ff_angle_target = st_cmd.f_FL_target_deg * 100;

    static float ff_angle_target_pre = 0;
    // velocity control
    if(b_servo_on_){
        // NLOG(info) << "[eddie] target rpm / angle: " << n_target_rpm_fl << " / " << ff_angle_target;
        uo_write1.un16_targetRPM = (uint16_t)(abs(n_target_rpm_f));
        uo_write1.un8_none = 0;
        uo_write1.n16_target_angle = (int16_t)(ff_angle_target);
        uo_write1.un8_interlock = 1;
        uo_write1.un8_direction = n_target_rpm_f < 0 ? 2 : 1;
    }
    else {
        LOG_INFO("NOT_SERVO_ON");
        uo_write1.un16_targetRPM = 0;
        uo_write1.un8_none = 0;
        uo_write1.n16_target_angle = (int16_t)(ff_angle_target_pre);
        uo_write1.un8_interlock = 1;
        uo_write1.un8_direction = n_target_rpm_f < 0 ? 2 : 1;
    }

    motor_msgs::MotorData data_1, data_2;

    data_1.input_velocity = st_cmd.f_FL_target_rpm;
    data_2.input_angle = st_cmd.f_FL_target_deg;
    
    UpdateMotorData(static_cast<int>(MotorID::FL_TRACTION), data_1, PDO_ID::RPDO1);  // RPDO1 (타겟 RPM)
    UpdateMotorData(static_cast<int>(MotorID::FL_STEER), data_2, PDO_ID::RPDO1);  // RPDO1 (타겟 RPM)
    // Can_driver::Write(
    //     CanID::CAN_RPDO1_ID + st_interface_param_.st_can_param.n_FL_traction_can_id, uo_write1.byte_space, 8);
    Can_driver::Write(0x223, uo_write1.byte_space, 8);
    // std::this_thread::sleep_for(std::chrono::microseconds(10));
    
    ff_angle_target_pre = ff_angle_target;
    // Can_driver::Write(0x080, 0, 0, 0, 0, 0, 0, 0, 0);
}

void Can_driver_curtis_sd::SetMotorIFGain(string& str_data)
{
    LOG_INFO("SET SetMotorIFGain");
}

void Can_driver_curtis_sd::SetMotorGain(string& str_data)
{
    LOG_INFO("SET SetMotorGain");
    try
    {
        istringstream iss(str_data);  // istringstream에 str을 담는다.
        string buffer;  // 구분자를 기준으로 절삭된 문자열이 담겨지는 버퍼
        vector<string> result;
        while (getline(iss, buffer, '/')) {
            result.push_back(buffer);  // 절삭된 문자열을 vector에 저장
        }
        if (result.size() != 10) {
            str_data = "";
            LOG_ERROR("SET GAIN TOPIC FAIL");
            return;
        }
        int n_fl_gain_p = stoi(result[0]);
        int n_fl_gain_i = stoi(result[1]);
        int n_fl_gain_ff = stoi(result[2]);
        int n_fl_gain_ff_acc = stoi(result[3]);
        int n_fl_cut_off = stoi(result[4]);

        int n_rr_gain_p = stoi(result[5]);
        int n_rr_gain_i = stoi(result[6]);
        int n_rr_gain_ff = stoi(result[7]);
        int n_rr_gain_ff_acc = stoi(result[8]);
        int n_rr_cut_off = stoi(result[9]);

        int n_FL_node_id = st_interface_param_.st_can_param.n_FL_traction_can_id;
        int n_RR_node_id = st_interface_param_.st_can_param.n_RR_traction_can_id;

        CAN_CURTIS_SD_SDO_READ uo_sdo_response;

        // get
        if(n_fl_gain_p == 0 && n_fl_gain_i == 0 && n_rr_gain_p == 0 && n_rr_gain_i == 0)
        {
            uo_sdo_response = SendSDOMsg(n_FL_node_id, 0x40, 0x30A2, 0x01, 0x00);
            if (uo_sdo_response.n8_command == 0x80) {
                NLOG(error) << "node : " << n_FL_node_id << " Get P Gain Fail";
                return;
            }
            int n_FL_current_gain_p = uo_sdo_response.n32_data;
            uo_sdo_response = SendSDOMsg(n_FL_node_id, 0x40, 0x30A2, 0x02, 0x00);
            if (uo_sdo_response.n8_command == 0x80) {
                NLOG(error) << "node : " << n_FL_node_id << " Get I Gain Fail";
                return;
            }
            int n_FL_current_gain_i = uo_sdo_response.n32_data;

            uo_sdo_response = SendSDOMsg(n_FL_node_id, 0x40, 0x30A2, 0x03, 0x00);
            if (uo_sdo_response.n8_command == 0x80) {
                NLOG(error) << "node : " << n_FL_node_id << " Get FF Gain Fail";
                return;
            }
            int n_FL_current_gain_ff = uo_sdo_response.n32_data;
            
            uo_sdo_response = SendSDOMsg(n_FL_node_id, 0x40, 0x30A2, 0x04, 0x00);
            if (uo_sdo_response.n8_command == 0x80) {
                NLOG(error) << "node : " << n_FL_node_id << " Get FF ACC Gain Fail";
                return;
            }
            int n_FL_current_gain_ff_acc = uo_sdo_response.n32_data;

            uo_sdo_response = SendSDOMsg(n_FL_node_id, 0x40, 0x30A2, 0x05, 0x00);
            if (uo_sdo_response.n8_command == 0x80) {
                NLOG(error) << "node : " << n_FL_node_id << " Get cut off Fail";
                return;
            }
            int n_FL_current_gain_cut_off = uo_sdo_response.n32_data;

            uo_sdo_response = SendSDOMsg(n_RR_node_id, 0x40, 0x30A2, 0x01, 0x00);
            if (uo_sdo_response.n8_command == 0x80) {
                NLOG(error) << "node : " << n_RR_node_id << " Get P Gain Fail";
                return;
            }
            int n_RR_current_gain_p = uo_sdo_response.n32_data;
            uo_sdo_response = SendSDOMsg(n_RR_node_id, 0x40, 0x30A2, 0x02, 0x00);
            if (uo_sdo_response.n8_command == 0x80) {
                NLOG(error) << "node : " << n_RR_node_id << " Get I Gain Fail";
                return;
            }
            int n_RR_current_gain_i = uo_sdo_response.n32_data;

            uo_sdo_response = SendSDOMsg(n_RR_node_id, 0x40, 0x30A2, 0x03, 0x00);
            if (uo_sdo_response.n8_command == 0x80) {
                NLOG(error) << "node : " << n_RR_node_id << " Get FF Gain Fail";
                return;
            }
            int n_RR_current_gain_ff = uo_sdo_response.n32_data;
            
            uo_sdo_response = SendSDOMsg(n_RR_node_id, 0x40, 0x30A2, 0x04, 0x00);
            if (uo_sdo_response.n8_command == 0x80) {
                NLOG(error) << "node : " << n_RR_node_id << " Get FF ACC Gain Fail";
                return;
            }
            int n_RR_current_gain_ff_acc = uo_sdo_response.n32_data;

            uo_sdo_response = SendSDOMsg(n_RR_node_id, 0x40, 0x30A2, 0x05, 0x00);
            if (uo_sdo_response.n8_command == 0x80) {
                NLOG(error) << "node : " << n_RR_node_id << " Get cut off Fail";
                return;
            }
            int n_RR_current_gain_cut_off = uo_sdo_response.n32_data;
            
            NLOG(info) << "Get GAIN FL P : " << n_FL_current_gain_p << ", I : " << n_FL_current_gain_i << ", " << n_FL_current_gain_ff << ", " << n_FL_current_gain_ff_acc << ", " << n_FL_current_gain_cut_off
                        << ", RR P : " << n_RR_current_gain_p << ", I : " << n_RR_current_gain_i << ", " << n_RR_current_gain_ff << ", " << n_RR_current_gain_ff_acc << ", " << n_RR_current_gain_cut_off;
        }
        // set
        else
        {
            NLOG(info) << "Set FL GAIN value : " << n_fl_gain_p << ", " << n_fl_gain_i << ", " << n_fl_gain_ff << ", " << n_fl_gain_ff_acc << ", " << n_fl_cut_off;
            
            if(n_fl_gain_p){
                uo_sdo_response = SendSDOMsg(n_FL_node_id, 0x23, 0x30A2, 0x01, n_fl_gain_p);
                if (uo_sdo_response.n8_command == 0x80) {
                    NLOG(error) << "node : " << n_FL_node_id << " Set P Gain Fail";
                    return;
                }
            }
            if(n_fl_gain_i){
                uo_sdo_response = SendSDOMsg(n_FL_node_id, 0x23, 0x30A2, 0x02, n_fl_gain_i);
                if (uo_sdo_response.n8_command == 0x80) {
                    NLOG(error) << "node : " << n_FL_node_id << " Set I Gain Fail";
                    return;
                }
            }
            if(n_fl_gain_ff){
                uo_sdo_response = SendSDOMsg(n_FL_node_id, 0x23, 0x30A2, 0x03, n_fl_gain_ff);
                if (uo_sdo_response.n8_command == 0x80) {
                    NLOG(error) << "node : " << n_FL_node_id << " Set ff Gain Fail";
                    return;
                }
            }
            if(n_fl_gain_ff_acc){
                uo_sdo_response = SendSDOMsg(n_FL_node_id, 0x23, 0x30A2, 0x04, n_fl_gain_ff_acc);
                if (uo_sdo_response.n8_command == 0x80) {
                    NLOG(error) << "node : " << n_FL_node_id << " Set ff acc Fail";
                    return;
                }
            }
            if(n_fl_cut_off){
                uo_sdo_response = SendSDOMsg(n_FL_node_id, 0x2b, 0x30A2, 0x05, n_fl_cut_off);
                if (uo_sdo_response.n8_command == 0x80) {
                    NLOG(error) << "node : " << n_FL_node_id << " Set cut off Fail";
                    return;
                }
            }

                
            NLOG(info) << "Set RR GAIN value : " << n_rr_gain_p << ", " << n_rr_gain_i << ", " << n_rr_gain_ff << ", " << n_rr_gain_ff_acc << ", " << n_rr_cut_off;

            if(n_rr_gain_p){
                uo_sdo_response = SendSDOMsg(n_RR_node_id, 0x23, 0x30A2, 0x01, n_rr_gain_p);
                if (uo_sdo_response.n8_command == 0x80) {
                    NLOG(error) << "node : " << n_RR_node_id << " Set P Gain Fail";
                    return;
                }
            }
            if(n_rr_gain_i){
                uo_sdo_response = SendSDOMsg(n_RR_node_id, 0x23, 0x30A2, 0x02, n_rr_gain_i);
                if (uo_sdo_response.n8_command == 0x80) {
                    NLOG(error) << "node : " << n_RR_node_id << " Set I Gain Fail";
                    return;
                }
            }

            if(n_rr_gain_ff){
                uo_sdo_response = SendSDOMsg(n_RR_node_id, 0x23, 0x30A2, 0x03, n_rr_gain_ff);
                if (uo_sdo_response.n8_command == 0x80) {
                    NLOG(error) << "node : " << n_RR_node_id << " Set ff Gain Fail";
                    return;
                }
            }
            if(n_rr_gain_ff_acc){
                uo_sdo_response = SendSDOMsg(n_RR_node_id, 0x23, 0x30A2, 0x04, n_rr_gain_ff_acc);
                if (uo_sdo_response.n8_command == 0x80) {
                    NLOG(error) << "node : " << n_RR_node_id << " Set ff acc Fail";
                    return;
                }
            }
            if(n_rr_cut_off){
                uo_sdo_response = SendSDOMsg(n_RR_node_id, 0x2b, 0x30A2, 0x05, n_rr_cut_off);
                if (uo_sdo_response.n8_command == 0x80) {
                    NLOG(error) << "node : " << n_RR_node_id << " Set cut off Fail";
                    return;
                }
            }
            uo_sdo_response = SendSDOMsg(n_FL_node_id, 0x23, 0x1010, 0x01, 0x65766173);
            uo_sdo_response = SendSDOMsg(n_RR_node_id, 0x23, 0x1010, 0x01, 0x65766173);

        }
    }
    catch (std::exception ex) {
        LOG_ERROR("%s", ex.what());
        return;
    }
}

void Can_driver_curtis_sd::CanCallback(const boost::any& any_type_var)
{
    can_frame o_canmsg = boost::any_cast<can_frame>(any_type_var);

    // Node ID 추출 (하위 7비트 사용)
    int node_id = o_canmsg.can_id & 0x7F;

    // FL traction TPDO1
    // if (o_canmsg.can_id == CanID::CAN_TPDO1_ID + st_interface_param_.st_can_param.n_FL_traction_can_id) {
    if (o_canmsg.can_id == 0x250) {
        CAN_CURTIS_SD_READ_1 uo_traction;
        memcpy(uo_traction.byte_space, o_canmsg.data, 8);

        motor_msgs::MotorData data_tr;
        float f_feedback_rpm = (float)(uo_traction.n16_motor_rpm);

        data_tr.error_code = (int)uo_traction.un8_error;
        data_tr.current = (float)uo_traction.un16_current;
        data_tr.voltage = (float)uo_traction.un16_voltage;
        data_tr.feedback_velocity = f_feedback_rpm;
        
        UpdateMotorData(static_cast<int>(MotorID::FL_TRACTION), data_tr, PDO_ID::TPDO1);  // TPDO1 (주행부 rpm, 전류, 전압, 에러 코드)
    }

    // FL steer TPDO1
    // if (o_canmsg.can_id == CanID::CAN_TPDO1_ID + st_interface_param_.st_can_param.n_FL_traction_can_id) {
    if (o_canmsg.can_id == 0x251) {
        CAN_CURTIS_SD_READ_2 uo_steer;
        memcpy(uo_steer.byte_space, o_canmsg.data, 8);

        motor_msgs::MotorData data_st;

        float f_feedback_angle = (float)(uo_steer.n16_steer_angle) / 100;  // 스케일 100
        float f_error_code = (float)(uo_steer.un8_error);  // 에러 코드

        data_st.feedback_angle = f_feedback_angle;
        data_st.error_code = f_error_code;
        UpdateMotorData(static_cast<int>(MotorID::FL_STEER), data_st, PDO_ID::TPDO2);  // TPDO1 (조향부 각도, 에러)
    }

    // abs steer TPDO3
    if (o_canmsg.can_id == CanID::CAN_TPDO1_ID + st_interface_param_.st_can_param.n_FL_abssteer_can_id) {
        CAN_CURTIS_SD_READ_3 uo_fl_tpdo3;
        motor_msgs::MotorData data_abs_steer;
        memcpy(uo_fl_tpdo3.byte_space, o_canmsg.data, 2);
        float f_angle = uo_fl_tpdo3.n16_encoder_value;

        if (f_angle > 7200)
            f_angle -= 14400;
        f_angle /= -40;
        data_abs_steer.feedback_angle = f_angle;
        data_abs_steer.encoder = uo_fl_tpdo3.n16_encoder_value;  // 스케일 -40

        UpdateMotorData(static_cast<int>(MotorID::FL_ABS_STEER), data_abs_steer, PDO_ID::TPDO3);  // TPDO1 (절댓값 엔코더 값)
    }
}

void Can_driver_curtis_sd::sendControlWord(ControlWord command, int motor_id)
{
    // // CAN 프레임 생성
    // CAN_CURTIS_SD_WRITE_2 cmd;
    // int n_id = 0;
    // cmd.control_word = static_cast<uint16_t>(command);  // ControlWord 값을 설정
    // // cmd.n32_target_velocity = 0;
    // // CAN 메시지 전송
    // if(motor_id == MotorID::FL_TRACTION)
    //     n_id = st_interface_param_.st_can_param.n_FL_traction_can_id;
    // else if(motor_id == MotorID::FL_STEER)
    //     n_id = st_interface_param_.st_can_param.n_FL_steer_can_id;
    // Can_driver::Write(CanID::CAN_RPDO1_ID + n_id, cmd.byte_space, 3);  // CanID::CAN_RPDO1_ID + motor_id: 모터 ID에 맞는 CAN ID
    // std::this_thread::sleep_for(std::chrono::microseconds(100));
    // LOG_INFO("ControlWord 0x%x sent to Motor %d", cmd.control_word, motor_id);
}

void Can_driver_curtis_sd::MotorSwitchOn(int motor_id, int n_type)
{
    std::lock_guard<std::mutex> lock(func_mutex_);

    LOG_INFO("CanOpenInitialize start for Motor %d", motor_id);
    Can_driver::Write(0x080, 0, 0, 0, 0, 0, 0, 0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    MotorState current_state = STATE_SWITCH_ON_DISABLE;
    bool nmt_command_sent = false;  // NMT 명령이 한 번만 보내지도록 플래그 설정
    std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now();
    const std::chrono::milliseconds timeout_duration(10000);  // 타임아웃 10초
    int retry_count = 0;  // 에러 발생 시 재시도 횟수
    const int max_retries = 3;  // 최대 재시도 횟수

    LOG_INFO("Motor %d: Initializing...", motor_id);

    while (current_state != STATE_COMPLETE && current_state != STATE_TIMEOUT) {
        // 타임아웃 체크
        std::chrono::system_clock::time_point current_time = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_time = current_time - start_time;

        if (elapsed_time >= timeout_duration) {
            current_state = STATE_TIMEOUT;
            break;
        }

        // 모터 상태 데이터 가져오기
        motor_msgs::MotorData motor_data = getMotorData(motor_id);

        current_state = handleMotorControl(static_cast<MotorStatus>(motor_data.status));
        LOG_TRACE(
            "Motor %d data updated: Status = 0x%x, Error = 0x%x, Current = %.2f, Velocity = %.2f, Encoder = %d", motor_id,
            motor_data.status, motor_data.error_code, motor_data.current, motor_data.feedback_velocity, motor_data.encoder);

        // 에러 상태 체크 (초기화 도중에 발생하는 에러 처리)
        if (motor_data.is_error) {
            LOG_ERROR("Motor %d: Error encountered during initialization, status: 0x%x", motor_id, motor_data.status);
            current_state = STATE_FAULT;
        }
        LOG_INFO("###Motor %d: Current state: %d", motor_id, current_state);
        switch (current_state) {
            case STATE_SWITCH_ON_DISABLE:
                LOG_INFO("Motor %d: Switch on disabled, sending command...", motor_id);
                sendControlWord(CONTROL_WORD_SHUTDOWN, motor_id);  // ControlWord 사용
                current_state = STATE_READY_TO_SWITCH_ON;
                break;

            case STATE_READY_TO_SWITCH_ON:
                LOG_INFO("Motor %d: Ready to switch on, sending switch on command...", motor_id);
                sendControlWord(CONTROL_WORD_SWITCH_ON, motor_id);  // ControlWord 사용
                current_state = STATE_SWITCH_ON;
                break;

            case STATE_SWITCH_ON:
                LOG_INFO("Motor %d: Switch on complete. Enabling operation...", motor_id);
                sendControlWord(CONTROL_WORD_ENABLE_OPERATION, motor_id);  // ControlWord 사용
                current_state = STATE_OPERATION_ENABLE;
                break;

            case STATE_OPERATION_ENABLE:
                LOG_INFO("Motor %d: Operation enabled. Motor is now fully operational.", motor_id);
                current_state = STATE_COMPLETE;
                break;

            case STATE_QUICK_STOP_ACTIVE:
                LOG_INFO("Motor %d: Quick Stop active, sending Quick Stop command...", motor_id);
                sendControlWord(CONTROL_WORD_QUICK_STOP, motor_id);  // ControlWord 사용
                current_state = STATE_COMPLETE;
                break;

            case STATE_FAULT_REACTION_ACTIVE:
            case STATE_FAULT:
                LOG_WARNING("Motor %d: Fault detected. Resetting motor...", motor_id);
                sendControlWord(CONTROL_WORD_FAULT_RESET, motor_id);  // ControlWord 사용
                retry_count++;
                if (retry_count < max_retries) {
                    current_state = STATE_RESET_MOTOR;  // 재시도 후 초기화 상태로
                }
                else {
                    LOG_ERROR("Motor %d: Max retry attempts reached. Initialization failed.", motor_id);
                    current_state = STATE_TIMEOUT;  // 타임아웃 상태로 전환
                }
                break;

            case STATE_RESET_MOTOR:
                LOG_INFO("Motor %d: Resetting motor...", motor_id);
                sendControlWord(CONTROL_WORD_FAULT_RESET, motor_id);  // ControlWord 사용
                current_state = STATE_SWITCH_ON_DISABLE;  // 다시 초기화 상태로
                break;

            case STATE_TIMEOUT:
                LOG_ERROR("Motor %d: Initialization timed out.", motor_id);
                return;

            case STATE_COMPLETE:
                LOG_INFO("Motor %d: Initialization completed successfully.", motor_id);
                return;

            default:
                LOG_ERROR("Motor %d: Unknown state during initialization.", motor_id);
                current_state = STATE_SWITCH_ON_DISABLE;
                break;
        }

        // CPU 사용량을 줄이기 위해 루프마다 지연 시간 추가
        Can_driver::Write(0x080, 0, 0, 0, 0, 0, 0, 0, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (current_state == STATE_COMPLETE) {
        LOG_INFO("Motor %d initialization completed successfully.", motor_id);
    }
    else if (current_state == STATE_TIMEOUT) {
        LOG_ERROR("Motor %d initialization timed out.", motor_id);
    }
    else if (current_state == STATE_FAULT) {
        LOG_ERROR("Motor %d encountered an error during initialization.", motor_id);
    }
}

// 특정 모터 데이터를 가져오는 getter
motor_msgs::MotorData Can_driver_curtis_sd::getMotorData(int motor_id)
{
    std::lock_guard<std::mutex> lock(data_mutex_);  // 멀티스레드 안전
    return motor_data_[motor_id];
}
// 특정 모터 데이터를 설정하는 setter
void Can_driver_curtis_sd::SetMotorData(int motor_id, const motor_msgs::MotorData& data)
{
    std::lock_guard<std::mutex> lock(data_mutex_);  // 멀티스레드 안전
    motor_data_[motor_id] = data;
}

void Can_driver_curtis_sd::UpdateMotorData(int motor_id, const motor_msgs::MotorData& update_data, int pdo_type)
{
    motor_msgs::MotorData data = getMotorData(motor_id);  // 해당 모터의 데이터를 가져옴

    data.motor_id = motor_id;
    auto now = std::chrono::system_clock::now();
    data.last_update_time = std::chrono::duration<double>(now.time_since_epoch()).count();

    if (pdo_type == PDO_ID::TPDO1) {  // TPDO1 처리 (Traction)
        data.feedback_velocity = update_data.feedback_velocity;  
        data.current = update_data.current;  
        data.voltage = update_data.voltage;  
        data.error_code = update_data.error_code;  // 에러 코드
        if (data.error_code != 0) {
            data.is_error = true;
        }
        else {
            data.is_error = false;
        }
    }
    else if (pdo_type == PDO_ID::TPDO2) {  // TPDO2 처리 (조향)
        data.feedback_angle = update_data.feedback_angle;  // 조향부 각도
        data.error_code = update_data.error_code;  // 에러 코드
        if (data.error_code != 0) {
            data.is_error = true;
        }
        else {
            data.is_error = false;
        }
    }
    else if (pdo_type == PDO_ID::TPDO3) {  // TPDO2 처리 (속도, 전류)
        data.feedback_angle = update_data.feedback_angle;  // 조향부 각도
        data.encoder = update_data.encoder;
    }
    else if (pdo_type == PDO_ID::RPDO1) {  // TPDO3 처리 (엔코더)
        data.input_velocity = update_data.input_velocity;
        data.input_angle = update_data.input_angle;  // 타겟 각도
    }
    SetMotorData(motor_id, data);
}

void Can_driver_curtis_sd::ProcessNmtMessage(int motor_id, const can_frame& o_canmsg)
{
    // NMT 메시지의 첫 번째 바이트는 상태를 나타냅니다.
    int nmt_state = o_canmsg.data[0];  // NMT 상태 코드

    motor_msgs::MotorData data = getMotorData(motor_id);  // 해당 모터의 데이터를 가져옴
    data.bus_state = nmt_state;  // 버스 상태 업데이트

    // 상태에 따라 로그 출력 (필요에 따라 추가적인 동작 수행)
    switch (nmt_state) {
        case NMT_BOOTUP:
            LOG_INFO("Motor %d NMT Bootup", motor_id);
            break;
        case NMT_STOPPED:
            LOG_INFO("Motor %d NMT Stopped", motor_id);
            break;
        case NMT_OPERATIONAL:
            LOG_INFO("Motor %d NMT Operational", motor_id);
            break;
        case NMT_PRE_OPERATIONAL:
            LOG_INFO("Motor %d NMT Pre-Operational", motor_id);
            break;
        default:
            LOG_WARNING("Motor %d Unknown NMT State: 0x%x", motor_id, nmt_state);
            break;
    }

    // 업데이트된 데이터를 설정
    SetMotorData(motor_id, data);
}

void Can_driver_curtis_sd::ResetMotorError(int motor_id)
{
    // LOG_INFO("Resetting error for Motor %d", motor_id);
    // int n_id = 0;
    // // 에러 리셋 명령 전송
    // CAN_CURTIS_SD_WRITE_2 reset_cmd;
    // reset_cmd.control_word = uint16_t(0x80);  // 에러 리셋 명령

    // if(motor_id == MotorID::FL_TRACTION)
    //     n_id = st_interface_param_.st_can_param.n_FL_traction_can_id;
    // else if(motor_id == MotorID::FL_STEER)
    //     n_id = st_interface_param_.st_can_param.n_FL_steer_can_id;
    // Can_driver::Write(CanID::CAN_RPDO1_ID + motor_id, reset_cmd.byte_space, 3);
    can_frame Message;
    Message.can_id = 0;
    Message.can_dlc = 2;
    Message.data[0] = NMTCommand::RESET_NODE;
    Message.data[1] = 0x00;
    Can_driver::Write(Message);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 에러 리셋 후 대기
}

bool Can_driver_curtis_sd::IsInitializing()
{
    return !b_init_;
}

CAN_CURTIS_SD_SDO_READ Can_driver_curtis_sd::SendSDOMsg(int node_id, int command, int index, int sub_index, int data)
{
    CAN_CURTIS_SD_SDO_WRITE o_sdo_write;
    o_sdo_write.n8_command = command;
    o_sdo_write.n16_index = index;
    o_sdo_write.n8_subindex = sub_index;
    o_sdo_write.n32_data = data;
    Can_driver::Write(CanID::CAN_SDO_REQUEST_ID + node_id, o_sdo_write.byte_space, 8);
    try {
        auto can_frame = sdo_response_manager_.waitForResponse(CanID::CAN_SDO_RESPONSE_ID + node_id, 1000);
        CAN_CURTIS_SD_SDO_READ uo_sdo_read;
        memcpy(uo_sdo_read.byte_space, can_frame.data, 8);
        return uo_sdo_read;
    }
    catch (std::exception& e) {
        NLOG(error) << "error occurred. " << e.what();
        // return false;
        CAN_CURTIS_SD_SDO_READ uo_sdo_read;
        return uo_sdo_read;
    }
}

std::map<int16_t, motor_msgs::MotorData> Can_driver_curtis_sd::GetMotorData()
{
    std::lock_guard<std::mutex> lock(data_mutex_);  // 멀티스레드 안전
    return motor_data_;
}