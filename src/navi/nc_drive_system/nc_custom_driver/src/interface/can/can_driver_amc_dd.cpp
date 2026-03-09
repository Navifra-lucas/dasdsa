
#include "interface/can/can_driver_amc_dd.hpp"

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

void Can_driver_amc_dd::Stop()
{
    LOG_INFO("Stop");

    CAN_AMC_DD_WRITE_2 uo_fl_write_2, uo_rr_write_2;

    uo_fl_write_2.n32_target_velocity = int32_t(0);
    uo_rr_write_2.n32_target_velocity = int32_t(0);

    // target 0
    Can_driver::Write(
        CanID::CAN_RPDO3_ID + st_interface_param_.st_can_param.n_FL_traction_can_id, uo_fl_write_2.byte_space, 4);
    Can_driver::Write(
        CanID::CAN_RPDO3_ID + st_interface_param_.st_can_param.n_RR_traction_can_id, uo_fl_write_2.byte_space, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    can_frame Message;
    Message.can_id = 0;
    Message.can_dlc = 2;
    Message.data[0] = NMTCommand::RESET_NODE;
    Message.data[1] = 0x00;
    Can_driver::Write(Message);
}

void Can_driver_amc_dd::Initialize()
{
    b_init_ = false;
    Can_driver::RegisteCallbackFunc("CanCallback", std::bind(&Can_driver_amc_dd::CanCallback, this, std::placeholders::_1));
    can_frame Message;
    Message.can_id = 0;
    Message.can_dlc = 2;
    Message.data[0] = NMTCommand::RESET_NODE;
    Message.data[1] = st_interface_param_.st_can_param.n_FL_traction_can_id;
    try{
        Can_driver::Write(Message);
        nmt_response_manager_.waitForResponse(
            CanID::CAN_NMT_STATUS_ID + st_interface_param_.st_can_param.n_FL_traction_can_id, 10000);
        Message.data[1] = st_interface_param_.st_can_param.n_RR_traction_can_id;
        Can_driver::Write(Message);
        nmt_response_manager_.waitForResponse(
            CanID::CAN_NMT_STATUS_ID + st_interface_param_.st_can_param.n_RR_traction_can_id, 10000);
    }
    catch(const std::exception& ex) {
        LOG_ERROR("%s", ex.what());
    }
    // pdo 맵핑은 윈도우 툴 사용해서 함.
    Message.can_id = 0;
    Message.can_dlc = 2;
    Message.data[0] = NMTCommand::OPERATIONAL;
    Message.data[1] = 0x00;
    Can_driver::Write(Message);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));  // NMT 명령 후 짧은 지연 필요
    
    MotorSwitchOn(MotorID::FL_TRACTION, 3);
    MotorSwitchOn(MotorID::RR_TRACTION, 3);

    auto now = std::chrono::system_clock::now();
    tp_fl_traction_ = now;
    tp_rr_traction_ = now;
    // Can_driver::Write(0x080, 0, 0, 0, 0, 0, 0, 0, 0);
    b_init_ = true;
}

void Can_driver_amc_dd::ControlOff(bool data)
{
   // b_control_off_ = data;
   if (data) {
    LOG_INFO("SET ControlOff ON");
    }
    if (!data) {
        LOG_INFO("SET ControlOff OFF");
    }
}

void Can_driver_amc_dd::ReInitializeCheck()
{
    // Activate emergency shutdown byte0 = 0x23 Index : 0x200C, sub : 0x00
}

void Can_driver_amc_dd::MotorDriverInit(string& str_data)
{
    // Activate emergency shutdown byte0 = 0x23 Index : 0x200C, sub : 0x00
    LOG_INFO("Motor Reinit Start");
    b_init_ = false;

    Can_driver::uninit();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));  // NMT 명령 후 짧은 지연 필요

    can_frame Message;
    Message.can_id = 0;
    Message.can_dlc = 2;
    Message.data[0] = NMTCommand::RESET_NODE;
    Message.data[1] = st_interface_param_.st_can_param.n_FL_traction_can_id;
    try {
        Can_driver::Write(Message);
        nmt_response_manager_.waitForResponse(
            CanID::CAN_NMT_STATUS_ID + st_interface_param_.st_can_param.n_FL_traction_can_id, 10000);
        Message.data[1] = st_interface_param_.st_can_param.n_RR_traction_can_id;
        Can_driver::Write(Message);
        nmt_response_manager_.waitForResponse(
            CanID::CAN_NMT_STATUS_ID + st_interface_param_.st_can_param.n_RR_traction_can_id, 10000);
    }
    catch(const std::exception& ex){
        LOG_ERROR("%s", ex.what());
    }

    // pdo 맵핑은 윈도우 툴 사용해서 함.
    Message.can_id = 0;
    Message.can_dlc = 2;
    Message.data[0] = NMTCommand::OPERATIONAL;  // Operational NMT
    Message.data[1] = 0x00;
    Can_driver::Write(Message);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));  // NMT 명령 후 짧은 지연 필요

    MotorSwitchOn(MotorID::FL_TRACTION, 3);
    MotorSwitchOn(MotorID::RR_TRACTION, 3);

    LOG_INFO("Motor Reinit Finish");

    auto now = std::chrono::system_clock::now();
    tp_fl_traction_ = now;
    tp_rr_traction_ = now;

    // Can_driver::Write(0x080, 0, 0, 0, 0, 0, 0, 0, 0);
    b_init_ = true;
}

void Can_driver_amc_dd::MotorErrorReset(std::string& str_data)
{
    LOG_INFO("Motor Reset Start");
    MotorSwitchOn(MotorID::FL_TRACTION, 3);
    MotorSwitchOn(MotorID::RR_TRACTION, 3);
    LOG_INFO("Motor Reset Finish");
}

void Can_driver_amc_dd::Write(const Wheel_Cmd_t& st_cmd)
{
    // NLOG(info)<<"Pcan_driver_samyang_dd Write";
    static int n_break_count = 0;
    if (!b_init_)
        return;

    int n_target_rpm_fl = 0;
    int n_target_rpm_rr = 0;
    // if (n_break_count >= 10) {
        // n_break_count = 10;
    n_target_rpm_fl = st_cmd.f_FL_target_rpm * 16384 * 4 / 60; // count/s
    n_target_rpm_rr = st_cmd.f_RR_target_rpm * 16384 * 4 / 60; // count/s
    // }
    motor_msgs::MotorData data_fl = getMotorData(MotorID::FL_TRACTION);  // 해당 모터의 데이터를 가져옴 
    motor_msgs::MotorData data_rr = getMotorData(MotorID::RR_TRACTION);  // 해당 모터의 데이터를 가져옴

    // velocity control
    CAN_AMC_DD_WRITE_2 uo_fl_write2, uo_rr_write2;
    CAN_AMC_DD_WRITE_1 uo_fl_write1, uo_rr_write1;

    if(b_servo_on_){
        uo_fl_write2.n32_target_velocity = n_target_rpm_fl;
        uo_rr_write2.n32_target_velocity = n_target_rpm_rr;
        uo_fl_write1.un16_control_word = uint16_t(15);
        uo_rr_write1.un16_control_word = uint16_t(15);
    }
    else {
        uo_fl_write2.n32_target_velocity = 0;
        uo_rr_write2.n32_target_velocity = 0;
        uo_fl_write1.un16_control_word = uint16_t(6);
        uo_rr_write1.un16_control_word = uint16_t(6);
    }

    motor_msgs::MotorData data_1, data_2;

    data_1.input_velocity= st_cmd.f_FL_target_rpm;
    data_2.input_velocity= st_cmd.f_RR_target_rpm;
    
    UpdateMotorData(static_cast<int>(MotorID::FL_TRACTION), data_1, PDO_ID::RPDO1);  // RPDO1 (타겟 RPM)
    UpdateMotorData(static_cast<int>(MotorID::RR_TRACTION), data_2, PDO_ID::RPDO1);  // RPDO1 (타겟 RPM)
    // control_word
    // Can_driver::Write(
    //     CanID::CAN_RPDO2_ID + st_interface_param_.st_can_param.n_FL_traction_can_id, uo_fl_write1.byte_space, 3);
    // std::this_thread::sleep_for(std::chrono::microseconds(100));
    // Can_driver::Write(
    //     CanID::CAN_RPDO2_ID + st_interface_param_.st_can_param.n_RR_traction_can_id, uo_rr_write1.byte_space, 3);
    // std::this_thread::sleep_for(std::chrono::microseconds(100));

    Can_driver::Write(
        CanID::CAN_RPDO2_ID + st_interface_param_.st_can_param.n_FL_traction_can_id, uo_fl_write1.byte_space, 4);
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    Can_driver::Write(
        CanID::CAN_RPDO2_ID + st_interface_param_.st_can_param.n_RR_traction_can_id, uo_rr_write1.byte_space, 4);
    // target_vel
    Can_driver::Write(
        CanID::CAN_RPDO3_ID + st_interface_param_.st_can_param.n_FL_traction_can_id, uo_fl_write2.byte_space, 4);
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    Can_driver::Write(
        CanID::CAN_RPDO3_ID + st_interface_param_.st_can_param.n_RR_traction_can_id, uo_rr_write2.byte_space, 4);
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    
    Can_driver::Write(0x080, 0, 0, 0, 0, 0, 0, 0, 0);
}

void Can_driver_amc_dd::SetMotorIFGain(string& str_data)
{
    LOG_INFO("SET SetMotorIFGain");
}

void Can_driver_amc_dd::SetMotorGain(string& str_data)
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

        CAN_AMC_DD_SDO_READ uo_sdo_response;

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

void Can_driver_amc_dd::CanCallback(const boost::any& any_type_var)
{
    can_frame o_canmsg = boost::any_cast<can_frame>(any_type_var);

    // Node ID 추출 (하위 7비트 사용)
    int node_id = o_canmsg.can_id & 0x7F;

    // SDO 응답 메시지 처리
    if ((o_canmsg.can_id & 0xFFF0) == CanID::CAN_SDO_RESPONSE_ID) {
        // ProcessSdoResponse(o_canmsg, static_cast<int>(node_id));
        sdo_response_manager_.setPromiseValue(o_canmsg.can_id, o_canmsg);
    }
    // NMT 상태 메시지 처리
    else if ((o_canmsg.can_id & 0xFFF0) == CanID::CAN_NMT_STATUS_ID) {
        nmt_response_manager_.setPromiseValue(o_canmsg.can_id, o_canmsg);
        ProcessNmtMessage(static_cast<int>(node_id), o_canmsg);
    }
    else {
        // std::cout << "Unknown CAN ID: 0x" << std::hex << o_canmsg.ID << std::dec << std::endl;
    }

    // FL traction TPDO1
    if (o_canmsg.can_id == CanID::CAN_TPDO1_ID + st_interface_param_.st_can_param.n_FL_traction_can_id) {
        CAN_AMC_DD_READ_1 uo_fl_tpdo1;
        memcpy(uo_fl_tpdo1.byte_space, o_canmsg.data, 8);

        motor_msgs::MotorData data_fl;
        int n_status = int(uo_fl_tpdo1.un16_status);
        int n_error_code = int(uo_fl_tpdo1.un16_error_code);
        float f_current = int(uo_fl_tpdo1.n32_traction_amps) / 8192; // 스케일 2^13
        data_fl.status = n_status;
        data_fl.error_code = n_error_code;
        data_fl.current = f_current;
        UpdateMotorData(static_cast<int>(MotorID::FL_TRACTION), data_fl, PDO_ID::TPDO1);  // TPDO1 (상태, 에러, 전류)
    }

    // RR traction TPDO1
    if (o_canmsg.can_id == CanID::CAN_TPDO1_ID + st_interface_param_.st_can_param.n_RR_traction_can_id) {
        CAN_AMC_DD_READ_1 uo_rr_tpdo1;
        memcpy(uo_rr_tpdo1.byte_space, o_canmsg.data, 8);

        motor_msgs::MotorData data_rr;
        int n_status = int(uo_rr_tpdo1.un16_status);
        int n_error_code = int(uo_rr_tpdo1.un16_error_code);
        float f_current = int(uo_rr_tpdo1.n32_traction_amps) / 8192; // 스케일 2^13;

        data_rr.status = n_status;
        data_rr.error_code = n_error_code;
        data_rr.current = f_current;
        UpdateMotorData(static_cast<int>(MotorID::RR_TRACTION), data_rr, PDO_ID::TPDO1);  // TPDO1 (상태, 에러, 전류)
    }

    // FL traction TPDO3
    if (o_canmsg.can_id == CanID::CAN_TPDO3_ID + st_interface_param_.st_can_param.n_FL_traction_can_id) {
        CAN_AMC_DD_READ_2 uo_fl_tpdo3;
        memcpy(uo_fl_tpdo3.byte_space, o_canmsg.data, 8);

        motor_msgs::MotorData data_fl;
        int n_encoder = int(uo_fl_tpdo3.n32_encoder);

        data_fl.encoder= n_encoder;
        UpdateMotorData(static_cast<int>(MotorID::FL_TRACTION), data_fl, PDO_ID::TPDO3);  // TPDO1 (상태, 에러, 전류)
    }

    // RR traction TPDO3
    if (o_canmsg.can_id == CanID::CAN_TPDO3_ID + st_interface_param_.st_can_param.n_RR_traction_can_id) {
        CAN_AMC_DD_READ_2 uo_rr_tpdo3;
        memcpy(uo_rr_tpdo3.byte_space, o_canmsg.data, 8);

        motor_msgs::MotorData data_rr;
        int n_encoder = int(uo_rr_tpdo3.n32_encoder);

        data_rr.encoder= n_encoder;
        UpdateMotorData(static_cast<int>(MotorID::RR_TRACTION), data_rr, PDO_ID::TPDO3);  // TPDO1 (상태, 에러, 전류)
    }

    // FL traction TPDO4
    if (o_canmsg.can_id == CanID::CAN_TPDO4_ID + st_interface_param_.st_can_param.n_FL_traction_can_id) {
        CAN_AMC_DD_READ_3 uo_fl_tpdo4;
        memcpy(uo_fl_tpdo4.byte_space, o_canmsg.data, 8);

        motor_msgs::MotorData data_fl;
        int n_velocity = int(uo_fl_tpdo4.n32_velocity) * 60 / (16384 * 4);

        data_fl.feedback_velocity= n_velocity;
        UpdateMotorData(static_cast<int>(MotorID::FL_TRACTION), data_fl, PDO_ID::TPDO4);  // TPDO1 (상태, 에러, 전류)
    }

    // RR traction TPDO4
    if (o_canmsg.can_id == CanID::CAN_TPDO4_ID + st_interface_param_.st_can_param.n_RR_traction_can_id) {
        CAN_AMC_DD_READ_3 uo_rr_tpdo4;
        memcpy(uo_rr_tpdo4.byte_space, o_canmsg.data, 8);

        motor_msgs::MotorData data_rr;
        int n_velocity = int(uo_rr_tpdo4.n32_velocity) * 60 / (16384 * 4);

        data_rr.feedback_velocity= n_velocity;
        UpdateMotorData(static_cast<int>(MotorID::RR_TRACTION), data_rr, PDO_ID::TPDO4);  // TPDO1 (상태, 에러, 전류)
    }
}

void Can_driver_amc_dd::sendControlWord(ControlWord command, int motor_id)
{
    // CAN 프레임 생성
    CAN_AMC_DD_WRITE_1 cmd;
    int n_id = 0;
    cmd.un16_control_word = static_cast<uint16_t>(command);  // ControlWord 값을 설정
    // cmd.n32_target_velocity = 0;
    // CAN 메시지 전송
    if(motor_id == MotorID::FL_TRACTION)
        n_id = st_interface_param_.st_can_param.n_FL_traction_can_id;
    else if(motor_id == MotorID::RR_TRACTION)
        n_id = st_interface_param_.st_can_param.n_RR_traction_can_id;
    Can_driver::Write(CanID::CAN_RPDO2_ID + n_id, cmd.byte_space, 3);  // CanID::CAN_RPDO1_ID + motor_id: 모터 ID에 맞는 CAN ID
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    LOG_INFO("ControlWord 0x%x sent to Motor %d", cmd.un16_control_word, motor_id);
}

void Can_driver_amc_dd::MotorSwitchOn(int motor_id, int n_type)
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
motor_msgs::MotorData Can_driver_amc_dd::getMotorData(int motor_id)
{
    std::lock_guard<std::mutex> lock(data_mutex_);  // 멀티스레드 안전
    return motor_data_[motor_id];
}
// 특정 모터 데이터를 설정하는 setter
void Can_driver_amc_dd::SetMotorData(int motor_id, const motor_msgs::MotorData& data)
{
    std::lock_guard<std::mutex> lock(data_mutex_);  // 멀티스레드 안전
    motor_data_[motor_id] = data;
}

void Can_driver_amc_dd::UpdateMotorData(int motor_id, const motor_msgs::MotorData& update_data, int pdo_type)
{
    motor_msgs::MotorData data = getMotorData(motor_id);  // 해당 모터의 데이터를 가져옴

    data.motor_id = motor_id;
    auto now = std::chrono::system_clock::now();
    data.last_update_time = std::chrono::duration<double>(now.time_since_epoch()).count();

    if (pdo_type == PDO_ID::TPDO1) {  // TPDO1 처리 (상태, 에러)
        data.status = update_data.status;
        data.error_code = update_data.error_code;
        data.current = update_data.current;  // mA -> A 변환
        if (data.error_code != 0) {
            data.is_error = true;
        }
        else {
            data.is_error = false;
        }
    }
    else if (pdo_type == PDO_ID::TPDO3) {  // TPDO2 처리 (속도, 전류)

        data.encoder = update_data.encoder;
    }
    else if (pdo_type == PDO_ID::TPDO4) {  // TPDO3 처리 (엔코더)
        data.feedback_velocity = update_data.feedback_velocity;
    }
    else if (pdo_type == PDO_ID::RPDO1) {  // TPDO3 처리 (엔코더)
        data.input_velocity = update_data.input_velocity;
    }
    SetMotorData(motor_id, data);
}

void Can_driver_amc_dd::ProcessNmtMessage(int motor_id, const can_frame& o_canmsg)
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

void Can_driver_amc_dd::ResetMotorError(int motor_id)
{
    LOG_INFO("Resetting error for Motor %d", motor_id);
    int n_id = 0;
    // 에러 리셋 명령 전송
    CAN_AMC_DD_WRITE_1 reset_cmd;
    reset_cmd.un16_control_word = uint16_t(0x80);  // 에러 리셋 명령

    if(motor_id == MotorID::FL_TRACTION)
        n_id = st_interface_param_.st_can_param.n_FL_traction_can_id;
    else if(motor_id == MotorID::RR_TRACTION)
        n_id = st_interface_param_.st_can_param.n_RR_traction_can_id;
    Can_driver::Write(CanID::CAN_RPDO2_ID + motor_id, reset_cmd.byte_space, 3);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 에러 리셋 후 대기
}

bool Can_driver_amc_dd::IsInitializing()
{
    return !b_init_;
}

CAN_AMC_DD_SDO_READ Can_driver_amc_dd::SendSDOMsg(int node_id, int command, int index, int sub_index, int data)
{
    CAN_AMC_DD_SDO_WRITE o_sdo_write;
    o_sdo_write.n8_command = command;
    o_sdo_write.n16_index = index;
    o_sdo_write.n8_subindex = sub_index;
    o_sdo_write.n32_data = data;
    Can_driver::Write(CanID::CAN_SDO_REQUEST_ID + node_id, o_sdo_write.byte_space, 8);
    try {
        auto can_frame = sdo_response_manager_.waitForResponse(CanID::CAN_SDO_RESPONSE_ID + node_id, 1000);
        CAN_AMC_DD_SDO_READ uo_sdo_read;
        memcpy(uo_sdo_read.byte_space, can_frame.data, 8);
        return uo_sdo_read;
    }
    catch (std::exception& e) {
        NLOG(error) << "error occurred. " << e.what();
        // return false;
        CAN_AMC_DD_SDO_READ uo_sdo_read;
        return uo_sdo_read;
    }
}

std::map<int16_t, motor_msgs::MotorData> Can_driver_amc_dd::GetMotorData()
{
    std::lock_guard<std::mutex> lock(data_mutex_);  // 멀티스레드 안전
    return motor_data_;
}