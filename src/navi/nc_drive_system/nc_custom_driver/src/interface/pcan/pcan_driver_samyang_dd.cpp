
#include "interface/pcan/pcan_driver_samyang_dd.hpp"

#include "core_msgs/MotorServoStatus.h"
#include "util/logger.hpp"

using namespace NaviFra;

// NMT 상태를 정의하는 enum
enum NMTState
{
    NMT_BOOTUP = 0x00,
    NMT_STOPPED = 0x04,
    NMT_OPERATIONAL = 0x05,
    NMT_PRE_OPERATIONAL = 0x7F
};

void Pcan_driver_samyang_dd::Stop()
{
    LOG_INFO("Stop");

    PCAN_SAMYANG_DD_RPDO_1 uo_fl_write, uo_rr_write;

    uo_fl_write.un32_target_velocity = int32_t(0);
    uo_rr_write.un32_target_velocity = int32_t(0);

    Pcan_driver::Write(
        CanID::CAN_RPDO1_ID + st_interface_param_.st_pcan_param.n_FL_traction_pcan_id, uo_fl_write.byte_space, 7);
    Pcan_driver::Write(
        CanID::CAN_RPDO1_ID + st_interface_param_.st_pcan_param.n_RR_traction_pcan_id, uo_rr_write.byte_space, 7);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    TPCANMsg Message;

    Message.ID = 0;
    Message.LEN = 2;
    Message.DATA[0] = NMTCommand::RESET_NODE;
    Message.DATA[1] = 0x00;
    Pcan_driver::Write(Message);
}

// void Pcan_driver_samyang_dd::SetBrakePolarity()
// {
//     PCAN_SAMYANG_DD_SDO_READ uo_sdo_response;
//     if(st_driver_param_.b_brake_polarity) {
//         NLOG(info) << "node : " << st_interface_param_.st_pcan_param.n_FL_traction_pcan_id << "Set FL brake polarity HIGH";
//         uo_sdo_response = SendSDOMsg(st_interface_param_.st_pcan_param.n_FL_traction_pcan_id, 0x22, 0x3150, 0x02, 0x0000);
//         if (uo_sdo_response.n8_command == 0x80) {
//             NLOG(error) << "node : " << st_interface_param_.st_pcan_param.n_FL_traction_pcan_id << "Set FL brake polarity FAIL";
//             return;
//         }
//         NLOG(info) << "node : " << st_interface_param_.st_pcan_param.n_RR_traction_pcan_id << "Set RR brake polarity HIGH";
//         uo_sdo_response = SendSDOMsg(st_interface_param_.st_pcan_param.n_RR_traction_pcan_id, 0x22, 0x3150, 0x02, 0x0000);
//         if (uo_sdo_response.n8_command == 0x80) {
//             NLOG(error) << "node : " << st_interface_param_.st_pcan_param.n_RR_traction_pcan_id << "Set RR brake polarity FAIL";
//             return;
//         }
        
//     }

//     else {
//         NLOG(info) << "node : " << st_interface_param_.st_pcan_param.n_FL_traction_pcan_id << "Set FL brake polarity LOW";
//         uo_sdo_response = SendSDOMsg(st_interface_param_.st_pcan_param.n_FL_traction_pcan_id, 0x22, 0x3150, 0x02, 0x0001);
//         if (uo_sdo_response.n8_command == 0x80) {
//             NLOG(error) << "node : " << st_interface_param_.st_pcan_param.n_FL_traction_pcan_id << "Set FL brake polarity FAIL";
//             return;
//         }

//         NLOG(info) << "node : " << st_interface_param_.st_pcan_param.n_RR_traction_pcan_id << "Set RR brake polarity LOW";
//         uo_sdo_response = SendSDOMsg(st_interface_param_.st_pcan_param.n_RR_traction_pcan_id, 0x22, 0x3150, 0x02, 0x0001);
//         if (uo_sdo_response.n8_command == 0x80) {
//             NLOG(error) << "node : " << st_interface_param_.st_pcan_param.n_RR_traction_pcan_id << "Set RR brake polarity FAIL";
//             return;
//         }
//     }
// }

// void Pcan_driver_samyang_dd::SetBrakeCommand()
// {
//     if (!st_driver_param_.b_brake_command)
//     {
//         n_brake_on_ = 0x00010000;
//         n_brake_off_ = 0x00;
//     }
//     else
//     {
//         n_brake_on_ = 0x00;
//         n_brake_off_ = 0x00010000;
//     }

// }

// void Pcan_driver_samyang_dd::SetBrakeConfig()
// {   
//     SetBrakePolarity();
//     SetBrakeCommand();
// }

void Pcan_driver_samyang_dd::Initialize()
{
    b_init_ = false;
    Pcan_driver::RegisteCallbackFunc("PcanCallback", std::bind(&Pcan_driver_samyang_dd::PcanCallback, this, std::placeholders::_1));
    TPCANMsg Message;
    Message.ID = 0;
    Message.LEN = 2;
    Message.DATA[0] = NMTCommand::RESET_NODE;
    Message.DATA[1] = st_interface_param_.st_pcan_param.n_FL_traction_pcan_id;
    try{
        Pcan_driver::Write(Message);
        nmt_response_manager_.waitForResponse(
            CanID::CAN_NMT_STATUS_ID + st_interface_param_.st_pcan_param.n_FL_traction_pcan_id, 10000);
        Message.DATA[1] = st_interface_param_.st_pcan_param.n_RR_traction_pcan_id;
        Pcan_driver::Write(Message);
        nmt_response_manager_.waitForResponse(
            CanID::CAN_NMT_STATUS_ID + st_interface_param_.st_pcan_param.n_RR_traction_pcan_id, 10000);
    }
    catch(const std::exception& ex) {
        LOG_ERROR("%s", ex.what());
    }
    
    // SetBrakeConfig();

    DoRpdoMapping(st_interface_param_.st_pcan_param.n_FL_traction_pcan_id, 0xFF);
    DoRpdoMapping(st_interface_param_.st_pcan_param.n_RR_traction_pcan_id, 0xFF);
    DoTpdoMapping(st_interface_param_.st_pcan_param.n_FL_traction_pcan_id, 0x01);
    DoTpdoMapping(st_interface_param_.st_pcan_param.n_RR_traction_pcan_id, 0x01);

    Message.ID = 0;
    Message.LEN = 2;
    Message.DATA[0] = NMTCommand::OPERATIONAL;
    Message.DATA[1] = 0x00;
    Pcan_driver::Write(Message);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));  // NMT 명령 후 짧은 지연 필요

    MotorSwitchOn(st_interface_param_.st_pcan_param.n_FL_traction_pcan_id, 3);
    MotorSwitchOn(st_interface_param_.st_pcan_param.n_RR_traction_pcan_id, 3);

    auto now = std::chrono::system_clock::now();
    tp_fl_traction_ = now;
    tp_rr_traction_ = now;
    // Pcan_driver::Write(0x080, 0, 0, 0, 0, 0, 0, 0, 0);
    b_init_ = true;
}

void Pcan_driver_samyang_dd::DoRpdoMapping(int node_id, int transmission_type)
{
    PCAN_SAMYANG_DD_SDO_READ uo_sdo_response;
    NLOG(info) << "node : " << node_id << " DoRpdoMapping Start";
    /****************************/
    //      RPDO1 MAPPING       //
    /****************************/
    uo_sdo_response = SendSDOMsg(node_id, 0x2F, 0x1600, 0x00, 0x00);
    if (uo_sdo_response.n8_command == 0x80) {
        NLOG(error) << "node : " << node_id << " DoRpdo1Mapping Fail";
        return;
    }

    uo_sdo_response = SendSDOMsg(node_id, 0x23, 0x1600, 0x01, 0x60400010);
    if (uo_sdo_response.n8_command == 0x80) {
        NLOG(error) << "node : " << node_id << " DoRpdo1Mapping Fail";
        return;
    }

    uo_sdo_response = SendSDOMsg(node_id, 0x23, 0x1600, 0x02, 0x60600008);
    if (uo_sdo_response.n8_command == 0x80) {
        NLOG(error) << "node : " << node_id << " DoRpdo1Mapping Fail";
        return;
    }

    uo_sdo_response = SendSDOMsg(node_id, 0x23, 0x1600, 0x03, 0x60FF0020);
    if (uo_sdo_response.n8_command == 0x80) {
        NLOG(error) << "node : " << node_id << " DoRpdo1Mapping Fail";
        return;
    }

    uo_sdo_response = SendSDOMsg(node_id, 0x2F, 0x1400, 0x02, transmission_type);
    if (uo_sdo_response.n8_command == 0x80) {
        NLOG(error) << "node : " << node_id << " DoRpdo1Mapping Fail";
        return;
    }

    uo_sdo_response = SendSDOMsg(node_id, 0x23, 0x1400, 0x01, CanID::CAN_RPDO1_ID + node_id);
    if (uo_sdo_response.n8_command == 0x80) {
        NLOG(error) << "node : " << node_id << " DoRpdo1Mapping Fail";
        return;
    }

    uo_sdo_response = SendSDOMsg(node_id, 0x2F, 0x1600, 0x00, 0x03);
    if (uo_sdo_response.n8_command == 0x80) {
        NLOG(error) << "node : " << node_id << " DoRpdo1Mapping Fail";
        return;
    }
}

void Pcan_driver_samyang_dd::DoTpdoMapping(int node_id, int transmission_type)
{
    /****************************/
    //      TPDO1 MAPPING       //
    /****************************/
    PCAN_SAMYANG_DD_SDO_READ uo_sdo_response;
    NLOG(info) << "node : " << node_id << " DoTpdoMapping Start";
    uo_sdo_response = SendSDOMsg(node_id, 0x2F, 0x1A00, 0x00, 0x00);
    if (uo_sdo_response.n8_command == 0x80) {
        NLOG(error) << "node : " << node_id << " DoTpdo1Mapping Fail";
        return;
    }
    uo_sdo_response = SendSDOMsg(node_id, 0x23, 0x1A00, 0x01, 0x60410010);
    if (uo_sdo_response.n8_command == 0x80) {
        NLOG(error) << "node : " << node_id << " DoTpdo1Mapping Fail";
        return;
    }
    uo_sdo_response = SendSDOMsg(node_id, 0x23, 0x1A00, 0x02, 0x60610008);
    if (uo_sdo_response.n8_command == 0x80) {
        NLOG(error) << "node : " << node_id << " DoTpdo1Mapping Fail";
        return;
    }
    uo_sdo_response = SendSDOMsg(node_id, 0x2F, 0x1800, 0x02, transmission_type);
    if (uo_sdo_response.n8_command == 0x80) {
        NLOG(error) << "node : " << node_id << " DoTpdo1Mapping Fail";
        return;
    }
    if (transmission_type != 1) {
        uo_sdo_response = SendSDOMsg(node_id, 0x2B, 0x1800, 0x03, 0x32);
        if (uo_sdo_response.n8_command == 0x80) {
            NLOG(error) << "node : " << node_id << " DoTpdo1Mapping Fail";
            return;
        }
    }
    uo_sdo_response = SendSDOMsg(node_id, 0x23, 0x1800, 0x01, CanID::CAN_TPDO1_ID + node_id);
    if (uo_sdo_response.n8_command == 0x80) {
        NLOG(error) << "node : " << node_id << " DoTpdo1Mapping Fail";
        return;
    }

    uo_sdo_response = SendSDOMsg(node_id, 0x2F, 0x1A00, 0x00, 0x02);
    if (uo_sdo_response.n8_command == 0x80) {
        NLOG(error) << "node : " << node_id << " DoTpdo1Mapping Fail";
        return;
    }
    /****************************/
    //      TPDO2 MAPPING       //
    /****************************/
    uo_sdo_response = SendSDOMsg(node_id, 0x2F, 0x1A01, 0x00, 0x00);
    if (uo_sdo_response.n8_command == 0x80) {
        NLOG(error) << "node : " << node_id << " DoTpdo2Mapping Fail";
        return;
    }
    uo_sdo_response = SendSDOMsg(node_id, 0x23, 0x1A01, 0x01, 0x60640020);
    if (uo_sdo_response.n8_command == 0x80) {
        NLOG(error) << "node : " << node_id << " DoTpdo2Mapping Fail";
        return;
    }
    uo_sdo_response = SendSDOMsg(node_id, 0x23, 0x1A01, 0x02, 0x606C0020);
    if (uo_sdo_response.n8_command == 0x80) {
        NLOG(error) << "node : " << node_id << " DoTpdo1Mapping Fail";
        return;
    }
    uo_sdo_response = SendSDOMsg(node_id, 0x2F, 0x1801, 0x02, transmission_type);
    if (uo_sdo_response.n8_command == 0x80) {
        NLOG(error) << "node : " << node_id << " DoTpdo2Mapping Fail";
        return;
    }
    if (transmission_type != 1) {
        uo_sdo_response = SendSDOMsg(node_id, 0x2B, 0x1801, 0x03, 0x32);
        if (uo_sdo_response.n8_command == 0x80) {
            NLOG(error) << "node : " << node_id << " DoTpdo2Mapping Fail";
            return;
        }
    }
    uo_sdo_response = SendSDOMsg(node_id, 0x23, 0x1801, 0x01, CanID::CAN_TPDO2_ID + node_id);
    if (uo_sdo_response.n8_command == 0x80) {
        NLOG(error) << "node : " << node_id << " DoTpdo2Mapping Fail";
        return;
    }
    uo_sdo_response = SendSDOMsg(node_id, 0x2F, 0x1A01, 0x00, 0x02);
    if (uo_sdo_response.n8_command == 0x80) {
        NLOG(error) << "node : " << node_id << " DoTpdo2Mapping Fail";
        return;
    }
    /****************************/
    //      TPDO3 MAPPING       //
    /****************************/
    uo_sdo_response = SendSDOMsg(node_id, 0x2F, 0x1A02, 0x00, 0x00);
    if (uo_sdo_response.n8_command == 0x80) {
        NLOG(error) << "node : " << node_id << " DoTpdo3Mapping Fail";
        return;
    }
    uo_sdo_response = SendSDOMsg(node_id, 0x23, 0x1A02, 0x01, 0x60790020);
    if (uo_sdo_response.n8_command == 0x80) {
        NLOG(error) << "node : " << node_id << " DoTpdo3Mapping Fail";
        return;
    }
    uo_sdo_response = SendSDOMsg(node_id, 0x23, 0x1A02, 0x02, 0x60780010);
    if (uo_sdo_response.n8_command == 0x80) {
        NLOG(error) << "node : " << node_id << " DoTpdo3Mapping Fail";
        return;
    }
    uo_sdo_response = SendSDOMsg(node_id, 0x23, 0x1A02, 0x03, 0x603f0010);
    if (uo_sdo_response.n8_command == 0x80) {
        NLOG(error) << "node : " << node_id << " DoTpdo3Mapping Fail";
        return;
    }
    uo_sdo_response = SendSDOMsg(node_id, 0x2F, 0x1802, 0x02, transmission_type);
    if (uo_sdo_response.n8_command == 0x80) {
        NLOG(error) << "node : " << node_id << " DoTpdo3Mapping Fail";
        return;
    }
    if (transmission_type != 1) {
        uo_sdo_response = SendSDOMsg(node_id, 0x2B, 0x1802, 0x03, 0x32);
        if (uo_sdo_response.n8_command == 0x80) {
            NLOG(error) << "node : " << node_id << " DoTpdo3Mapping Fail";
            return;
        }
    }
    uo_sdo_response = SendSDOMsg(node_id, 0x23, 0x1802, 0x01, CanID::CAN_TPDO3_ID + node_id);
    if (uo_sdo_response.n8_command == 0x80) {
        NLOG(error) << "node : " << node_id << " DoTpdo3Mapping Fail";
        return;
    }
    uo_sdo_response = SendSDOMsg(node_id, 0x2F, 0x1A02, 0x00, 0x03);
    if (uo_sdo_response.n8_command == 0x80) {
        NLOG(error) << "node : " << node_id << " DoTpdo3Mapping Fail";
        return;
    }
    uo_sdo_response = SendSDOMsg(node_id, 0x2F, 0x1A03, 0x00, 0x00);
    if (uo_sdo_response.n8_command == 0x80) {
        NLOG(error) << "node : " << node_id << " DoTpdo4Mapping Fail";
        return;
    }
    uo_sdo_response = SendSDOMsg(node_id, 0x23, 0x1010, 0x01, 0x65766173);
    if (uo_sdo_response.n8_command == 0x80) {
        NLOG(error) << "node : " << node_id << " DoTpdoMapping Fail";
        return;
    }
    NLOG(info) << "node : " << node_id << " DoTpdoMapping Done";
}

void Pcan_driver_samyang_dd::ControlOff(bool data)
{
    // b_control_off_ = data;
    if (data) {
        LOG_INFO("SET ControlOff ON");
    }
    if (!data) {
        LOG_INFO("SET ControlOff OFF");
    }
}

void Pcan_driver_samyang_dd::ReInitializeCheck()
{
}

void Pcan_driver_samyang_dd::MotorDriverInit(std::string& str_data)
{
    // Activate emergency shutdown byte0 = 0x23 Index : 0x200C, sub : 0x00
    NLOG(info)<<"Motor Reinit Start";
    b_init_ = false;

    Pcan_driver::uninit();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));  // NMT 명령 후 짧은 지연 필요

    TPCANMsg Message;
    Message.ID = 0;
    Message.LEN = 2;
    Message.DATA[0] = NMTCommand::RESET_NODE;
    Message.DATA[1] = st_interface_param_.st_pcan_param.n_FL_traction_pcan_id;
    try {
        Pcan_driver::Write(Message);
        nmt_response_manager_.waitForResponse(
            CanID::CAN_NMT_STATUS_ID + st_interface_param_.st_pcan_param.n_FL_traction_pcan_id, 10000);
        Message.DATA[1] = st_interface_param_.st_pcan_param.n_RR_traction_pcan_id;
        Pcan_driver::Write(Message);
        nmt_response_manager_.waitForResponse(
            CanID::CAN_NMT_STATUS_ID + st_interface_param_.st_pcan_param.n_RR_traction_pcan_id, 10000);
    }
    catch(const std::exception& ex){
        LOG_ERROR("%s", ex.what());
    }

    DoRpdoMapping(st_interface_param_.st_pcan_param.n_FL_traction_pcan_id, 0xFF);
    DoRpdoMapping(st_interface_param_.st_pcan_param.n_RR_traction_pcan_id, 0xFF);
    DoTpdoMapping(st_interface_param_.st_pcan_param.n_FL_traction_pcan_id, 0xFF);
    DoTpdoMapping(st_interface_param_.st_pcan_param.n_RR_traction_pcan_id, 0xFF);

    Message.ID = 0;
    Message.LEN = 2;
    Message.DATA[0] = NMTCommand::OPERATIONAL;  // Operational NMT
    Message.DATA[1] = 0x00;
    Pcan_driver::Write(Message);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));  // NMT 명령 후 짧은 지연 필요

    MotorSwitchOn(st_interface_param_.st_pcan_param.n_FL_traction_pcan_id, 3);
    MotorSwitchOn(st_interface_param_.st_pcan_param.n_RR_traction_pcan_id, 3);

    NLOG(info)<<"Motor Reinit Finish";

    auto now = std::chrono::system_clock::now();
    tp_fl_traction_ = now;
    tp_rr_traction_ = now;

    // Pcan_driver::Write(0x080, 0, 0, 0, 0, 0, 0, 0, 0);
    b_init_ = true;
}

void Pcan_driver_samyang_dd::MotorErrorReset(std::string& str_data)
{
    LOG_INFO("Motor Reset Start");
    MotorSwitchOn(st_interface_param_.st_pcan_param.n_FL_traction_pcan_id, 3);
    MotorSwitchOn(st_interface_param_.st_pcan_param.n_RR_traction_pcan_id, 3);
    LOG_INFO("Motor Reset Finish");
}

// void Pcan_driver_samyang_dd::ControlBrake(bool data)
// {
//     NLOG(info)<<"BRAKE CONTROL";

//     motor_msgs::MotorData data_fl = getMotorData(1);  
//     motor_msgs::MotorData data_rr = getMotorData(2);  

//     PCAN_SAMYANG_DD_SDO_READ uo_sdo_response;
//     if (data) {
//         // NLOG(info)<<"BRAKE ON";
//         // uo_sdo_response = SendSDOMsg(st_interface_param_.st_pcan_param.n_FL_traction_pcan_id, 0x22, 0x60fe, 0x01, n_brake_on_); // General purpose A
//         // if (uo_sdo_response.n8_command == 0x80) {
//         //     NLOG(error) << "node : " << st_interface_param_.st_pcan_param.n_FL_traction_pcan_id << " Brake ON Fail";
//         //     return;
//         // }
//         // uo_sdo_response = SendSDOMsg(st_interface_param_.st_pcan_param.n_RR_traction_pcan_id, 0x22, 0x60fe, 0x01, n_brake_on_);
//         // if (uo_sdo_response.n8_command == 0x80) {
//         //     NLOG(error) << "node : " << st_interface_param_.st_pcan_param.n_RR_traction_pcan_id << " Brake ON Fail";
//         //     return;
//         // }
//         uo_sdo_response = SendSDOMsg(st_interface_param_.st_pcan_param.n_FL_traction_pcan_id, 0x2b, 0x6040, 0x00, 0x06);
//         if (uo_sdo_response.n8_command == 0x80) {
//             NLOG(error) << "node : " << st_interface_param_.st_pcan_param.n_FL_traction_pcan_id << " Servo ON Fail";
//             return;
//         }
//         uo_sdo_response = SendSDOMsg(st_interface_param_.st_pcan_param.n_RR_traction_pcan_id, 0x2b, 0x6040, 0x00, 0x06);
//         if (uo_sdo_response.n8_command == 0x80) {
//             NLOG(error) << "node : " << st_interface_param_.st_pcan_param.n_RR_traction_pcan_id << " Servo ON Fail";
//             return;
//         }
//         b_brake_release_ = false;
//     }
//     else {
//         b_brake_release_ = true;
//         NLOG(info)<<"BRAKE OFF";
//         uo_sdo_response = SendSDOMsg(st_interface_param_.st_pcan_param.n_FL_traction_pcan_id, 0x2b, 0x6040, 0x00, 0x00);
//         if (uo_sdo_response.n8_command == 0x80) {
//             NLOG(error) << "node : " << st_interface_param_.st_pcan_param.n_FL_traction_pcan_id << " Servo OFF Fail";
//             return;
//         }
//         uo_sdo_response = SendSDOMsg(st_interface_param_.st_pcan_param.n_RR_traction_pcan_id, 0x2b, 0x6040, 0x00, 0x00);
//         if (uo_sdo_response.n8_command == 0x80) {
//             NLOG(error) << "node : " << st_interface_param_.st_pcan_param.n_RR_traction_pcan_id << " Servo OFF Fail";
//             return;
//         }
//         uo_sdo_response = SendSDOMsg(st_interface_param_.st_pcan_param.n_FL_traction_pcan_id, 0x22, 0x60fe, 0x01, n_brake_off_);
//         if (uo_sdo_response.n8_command == 0x80) {
//             NLOG(error) << "node : " << st_interface_param_.st_pcan_param.n_FL_traction_pcan_id << " Brake OFF Fail";
//             return;
//         }
//         uo_sdo_response = SendSDOMsg(st_interface_param_.st_pcan_param.n_RR_traction_pcan_id, 0x22, 0x60fe, 0x01, n_brake_off_);
//         if (uo_sdo_response.n8_command == 0x80) {
//             NLOG(error) << "node : " << st_interface_param_.st_pcan_param.n_RR_traction_pcan_id << " Brake OFF Fail";
//             return;
//         }
//     }
// }

void Pcan_driver_samyang_dd::Write(const Wheel_Cmd_t& st_cmd)
{
    // NLOG(info)<<"Pcan_driver_samyang_dd Write";
    static int n_break_count = 0;
    if (!b_init_)
        return;

    int n_target_rpm_fl = 0;
    int n_target_rpm_rr = 0;
    // if (n_break_count >= 10) {
        // n_break_count = 10;
    n_target_rpm_fl = st_cmd.f_FL_target_rpm;
    n_target_rpm_rr = st_cmd.f_RR_target_rpm;
    // }
    motor_msgs::MotorData data_fl = getMotorData(1);  // 해당 모터의 데이터를 가져옴
    motor_msgs::MotorData data_rr = getMotorData(2);  // 해당 모터의 데이터를 가져옴

    // PCAN_SAMYANG_DD_SDO_READ uo_sdo_response;
    // if(st_driver_param_.b_brake_always_on) {
    //     uo_sdo_response = SendSDOMsg(st_interface_param_.st_pcan_param.st_pcan_qd.n_qd_FL_traction_pcan_id, 0x22, 0x60fe, 0x01, n_brake_off_);
    //     if (uo_sdo_response.n8_command == 0x80) {
    //         NLOG(error) << "node : " << st_interface_param_.st_pcan_param.st_pcan_qd.n_qd_FL_traction_pcan_id << " Brake OFF Fail";
    //         return;
    //     }
    //     uo_sdo_response = SendSDOMsg(st_interface_param_.st_pcan_param.st_pcan_qd.n_qd_RR_traction_pcan_id, 0x22, 0x60fe, 0x01, n_brake_off_);
    //     if (uo_sdo_response.n8_command == 0x80) {
    //         NLOG(error) << "node : " << st_interface_param_.st_pcan_param.st_pcan_qd.n_qd_RR_traction_pcan_id << " Brake OFF Fail";
    //         return;
    //     }   
    // }
    // else if(!b_brake_release_){
    //     if (!st_driver_param_.b_brake_always_on && st_cmd.f_FL_target_rpm == 0 && st_cmd.f_RR_target_rpm == 0 && abs(data_fl.feedback_velocity) < 5 && abs(data_rr.feedback_velocity) < 5) {
    //         n_break_count = 0;
    //         uo_sdo_response = SendSDOMsg(st_interface_param_.st_pcan_param.n_FL_traction_pcan_id, 0x22, 0x60fe, 0x01, n_brake_on_);
    //         if (uo_sdo_response.n8_command == 0x80) {
    //             NLOG(error) << "node : " << st_interface_param_.st_pcan_param.n_FL_traction_pcan_id << " Brake ON Fail";
    //             return;
    //         }
    //         uo_sdo_response = SendSDOMsg(st_interface_param_.st_pcan_param.n_RR_traction_pcan_id, 0x22, 0x60fe, 0x01, n_brake_on_);
    //         if (uo_sdo_response.n8_command == 0x80) {
    //             NLOG(error) << "node : " << st_interface_param_.st_pcan_param.n_RR_traction_pcan_id << " Brake ON Fail";
    //             return;
    //         }
    //     }
    //     else if (st_cmd.f_FL_target_rpm != 0 || st_cmd.f_RR_target_rpm != 0) {
    //         n_break_count++;
    //         uo_sdo_response = SendSDOMsg(st_interface_param_.st_pcan_param.n_FL_traction_pcan_id, 0x22, 0x60fe, 0x01, n_brake_off_);
    //         if (uo_sdo_response.n8_command == 0x80) {
    //             NLOG(error) << "node : " << st_interface_param_.st_pcan_param.n_FL_traction_pcan_id << " Brake OFF Fail";
    //             return;
    //         }
    //         uo_sdo_response = SendSDOMsg(st_interface_param_.st_pcan_param.n_RR_traction_pcan_id, 0x22, 0x60fe, 0x01, n_brake_off_);
    //         if (uo_sdo_response.n8_command == 0x80) {
    //             NLOG(error) << "node : " << st_interface_param_.st_pcan_param.n_RR_traction_pcan_id << " Brake OFF Fail";
    //             return;
    //         }
    //     }
    // }

    // velocity control
    PCAN_SAMYANG_DD_RPDO_1 uo_fl_write, uo_rr_write;

    uo_fl_write.un32_target_velocity = n_target_rpm_fl;
    uo_rr_write.un32_target_velocity = n_target_rpm_rr;
    uo_fl_write.un16_control_word = uint16_t(15);
    uo_rr_write.un16_control_word = uint16_t(15);

    motor_msgs::MotorData data_1, data_2;

    data_1.input_velocity = n_target_rpm_fl;
    data_2.input_velocity = n_target_rpm_rr;
    
    UpdateMotorData(static_cast<int>(MotorID::FL_TRACTION), data_1, PDO_ID::RPDO1);  // RPDO1 (타겟 RPM)
    UpdateMotorData(static_cast<int>(MotorID::RR_TRACTION), data_2, PDO_ID::RPDO1);  // RPDO1 (타겟 RPM)
    Pcan_driver::Write(
        CanID::CAN_RPDO1_ID + st_interface_param_.st_pcan_param.n_FL_traction_pcan_id, uo_fl_write.byte_space, 7);
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    Pcan_driver::Write(
        CanID::CAN_RPDO1_ID + st_interface_param_.st_pcan_param.n_RR_traction_pcan_id, uo_rr_write.byte_space, 7);
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    Pcan_driver::Write(0x080, 0, 0, 0, 0, 0, 0, 0, 0);
}

void Pcan_driver_samyang_dd::SetMotorIFGain(string& str_data)
{
    LOG_INFO("SET SetMotorIFGain");
}

void Pcan_driver_samyang_dd::SetMotorGain(string& str_data)
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

        int n_FL_node_id = st_interface_param_.st_pcan_param.n_FL_traction_pcan_id;
        int n_RR_node_id = st_interface_param_.st_pcan_param.n_RR_traction_pcan_id;

        PCAN_SAMYANG_DD_SDO_READ uo_sdo_response;

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

void Pcan_driver_samyang_dd::PcanCallback(const boost::any& any_type_var)
{
    TPCANMsg o_canmsg = boost::any_cast<TPCANMsg>(any_type_var);

    // Node ID 추출 (하위 7비트 사용)
    int node_id = o_canmsg.ID & 0x7F;

    // SDO 응답 메시지 처리
    if ((o_canmsg.ID & 0xFFF0) == CanID::CAN_SDO_RESPONSE_ID) {
        // ProcessSdoResponse(o_canmsg, static_cast<int>(node_id));
        sdo_response_manager_.setPromiseValue(o_canmsg.ID, o_canmsg);
    }
    // NMT 상태 메시지 처리
    else if ((o_canmsg.ID & 0xFFF0) == CanID::CAN_NMT_STATUS_ID) {
        nmt_response_manager_.setPromiseValue(o_canmsg.ID, o_canmsg);
        ProcessNmtMessage(static_cast<int>(node_id), o_canmsg);
    }
    else {
        // std::cout << "Unknown CAN ID: 0x" << std::hex << o_canmsg.ID << std::dec << std::endl;
    }

    // FL traction TPDO1
    if (o_canmsg.ID == CanID::CAN_TPDO1_ID + st_interface_param_.st_pcan_param.n_FL_traction_pcan_id) {
        PCAN_SAMYANG_DD_TPDO_1 uo_fl_tpdo1;
        memcpy(uo_fl_tpdo1.byte_space, o_canmsg.DATA, 8);

        motor_msgs::MotorData data_fl;
        int n_status = int(uo_fl_tpdo1.un16_status);
        // int n_error_code = int(uo_fl_tpdo1.un16_error_code);
        // float f_FL_current = int(uo_fl_tpdo1.n32_amps);

        data_fl.status = n_status;
        // data_fl.error_code = n_error_code;
        // data_fl.current = f_FL_current;
        UpdateMotorData(static_cast<int>(MotorID::FL_TRACTION), data_fl, PDO_ID::TPDO1);  // TPDO1 (상태, 에러, 전류)
    }
    
    // RR traction TPDO1
    if (o_canmsg.ID == CanID::CAN_TPDO1_ID + st_interface_param_.st_pcan_param.n_RR_traction_pcan_id) {
        PCAN_SAMYANG_DD_TPDO_1 uo_rr_tpdo1;
        memcpy(uo_rr_tpdo1.byte_space, o_canmsg.DATA, 8);

        motor_msgs::MotorData data_rr;
        int n_status = int(uo_rr_tpdo1.un16_status);
        // int n_error_code = int(uo_rr_tpdo1.un16_error_code);
        // float f_RR_current = int(uo_rr_tpdo1.n32_amps);

        data_rr.status = n_status;
        // data_rr.error_code = n_error_code;
        // data_rr.current = f_RR_current;
        UpdateMotorData(static_cast<int>(MotorID::RR_TRACTION), data_rr, PDO_ID::TPDO1);  // TPDO1 (상태, 에러, 전류)
    }

    // FL traction TPDO2
    if (o_canmsg.ID == CanID::CAN_TPDO2_ID + st_interface_param_.st_pcan_param.n_FL_traction_pcan_id) {
        PCAN_SAMYANG_DD_TPDO_2 uo_fl_tpdo2;
        memcpy(uo_fl_tpdo2.byte_space, o_canmsg.DATA, 8);

        motor_msgs::MotorData data_fl;
        float f_feedback_vel_fl = uo_fl_tpdo2.n32_velocity;
        int n32_encoder_fl = uo_fl_tpdo2.n32_encoder;

        data_fl.feedback_velocity = f_feedback_vel_fl;
        data_fl.encoder = n32_encoder_fl;
        UpdateMotorData(static_cast<int>(MotorID::FL_TRACTION), data_fl, PDO_ID::TPDO2);  // TPDO2 (속도, 엔코더)
    }

    // RR traction TPDO2
    if (o_canmsg.ID == CanID::CAN_TPDO2_ID + st_interface_param_.st_pcan_param.n_RR_traction_pcan_id) {
        PCAN_SAMYANG_DD_TPDO_2 uo_rr_tpdo2;
        memcpy(uo_rr_tpdo2.byte_space, o_canmsg.DATA, 8);

        motor_msgs::MotorData data_rr;
        float f_feedback_vel_rr = uo_rr_tpdo2.n32_velocity;
        int n32_encoder_rr = uo_rr_tpdo2.n32_encoder;

        data_rr.feedback_velocity = f_feedback_vel_rr;
        data_rr.encoder = n32_encoder_rr;
        UpdateMotorData(static_cast<int>(MotorID::RR_TRACTION), data_rr, PDO_ID::TPDO2);  // 2 (속도, 엔코더)
    }

    // FL traction TPDO3
    if (o_canmsg.ID == CanID::CAN_TPDO3_ID + st_interface_param_.st_pcan_param.n_FL_traction_pcan_id) {
        PCAN_SAMYANG_DD_TPDO_3 uo_fl_tpdo3;
        memcpy(uo_fl_tpdo3.byte_space, o_canmsg.DATA, 8);

        motor_msgs::MotorData data_fl;
        float f_FL_voltage = int(uo_fl_tpdo3.un32_voltage);
        float f_FL_current = int(uo_fl_tpdo3.n16_current);
        int n_error_code = int(uo_fl_tpdo3.un16_error_code);

        data_fl.voltage = f_FL_voltage;
        data_fl.current = f_FL_current;
        data_fl.error_code = n_error_code;
        UpdateMotorData(static_cast<int>(MotorID::FL_TRACTION), data_fl, PDO_ID::TPDO3);  // TPDO3 (속도, 엔코더)
    }

    // RR traction TPDO3
    if (o_canmsg.ID == CanID::CAN_TPDO3_ID + st_interface_param_.st_pcan_param.n_RR_traction_pcan_id) {
        PCAN_SAMYANG_DD_TPDO_3 uo_rr_tpdo3;
        memcpy(uo_rr_tpdo3.byte_space, o_canmsg.DATA, 8);

        motor_msgs::MotorData data_rr;
        float f_RR_voltage = int(uo_rr_tpdo3.un32_voltage);
        float f_RR_current = int(uo_rr_tpdo3.n16_current);
        int n_error_code = int(uo_rr_tpdo3.un16_error_code);

        data_rr.voltage = f_RR_voltage;
        data_rr.current = f_RR_current;
        data_rr.error_code = n_error_code;
        UpdateMotorData(static_cast<int>(MotorID::RR_TRACTION), data_rr, PDO_ID::TPDO3);  // 3 (속도, 엔코더)
    }
}

void Pcan_driver_samyang_dd::sendControlWord(ControlWord command, int motor_id)
{
    // CAN 프레임 생성
    PCAN_SAMYANG_DD_RPDO_1 cmd;
    cmd.un16_control_word = static_cast<uint16_t>(command);  // ControlWord 값을 설정
    // cmd.n32_target_velocity = 0;
    // CAN 메시지 전송
    Pcan_driver::Write(CanID::CAN_RPDO1_ID + motor_id, cmd.byte_space, 7);  // CanID::CAN_RPDO1_ID + motor_id: 모터 ID에 맞는 CAN ID
    LOG_INFO("ControlWord 0x%x sent to Motor %d", cmd.un16_control_word, motor_id);
}

void Pcan_driver_samyang_dd::MotorSwitchOn(int motor_id, int n_type)
{
    std::lock_guard<std::mutex> lock(func_mutex_);
    LOG_INFO("CanOpenInitialize start for Motor %d", motor_id);
    Pcan_driver::Write(0x080, 0, 0, 0, 0, 0, 0, 0, 0);
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

        switch (current_state) {
            case STATE_SWITCH_ON_DISABLE:
                LOG_INFO("Motor %d: Switch on disabled, sending command...", motor_id);
                sendControlWord(CONTROL_WORD_SHUTDOWN, motor_id);  // ControlWord 사용
                // current_state = STATE_READY_TO_SWITCH_ON;
                break;

            case STATE_READY_TO_SWITCH_ON:
                LOG_INFO("Motor %d: Ready to switch on, sending switch on command...", motor_id);
                sendControlWord(CONTROL_WORD_SWITCH_ON, motor_id);  // ControlWord 사용
                // current_state = STATE_SWITCH_ON;
                break;

            case STATE_SWITCH_ON:
                LOG_INFO("Motor %d: Switch on complete. Enabling operation...", motor_id);
                sendControlWord(CONTROL_WORD_ENABLE_OPERATION, motor_id);  // ControlWord 사용
                // current_state = STATE_OPERATION_ENABLE;
                break;

            case STATE_OPERATION_ENABLE:
                LOG_INFO("Motor %d: Operation enabled. Motor is now fully operational.", motor_id);
                current_state = STATE_COMPLETE;
                break;

            case STATE_QUICK_STOP_ACTIVE:
                LOG_INFO("Motor %d: Quick Stop active, sending Quick Stop command...", motor_id);
                sendControlWord(CONTROL_WORD_QUICK_STOP, motor_id);  // ControlWord 사용
                // current_state = STATE_COMPLETE;
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

        Pcan_driver::Write(0x080, 0, 0, 0, 0, 0, 0, 0, 0);
        // CPU 사용량을 줄이기 위해 루프마다 지연 시간 추가
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
motor_msgs::MotorData Pcan_driver_samyang_dd::getMotorData(int motor_id)
{
    std::lock_guard<std::mutex> lock(data_mutex_);  // 멀티스레드 안전
    return motor_data_[motor_id];
}

// 특정 모터 데이터를 설정하는 setter
void Pcan_driver_samyang_dd::SetMotorData(int motor_id, const motor_msgs::MotorData& data)
{
    std::lock_guard<std::mutex> lock(data_mutex_);  // 멀티스레드 안전
    motor_data_[motor_id] = data;
}

void Pcan_driver_samyang_dd::UpdateMotorData(int motor_id, const motor_msgs::MotorData& update_data, int pdo_type)
{
    motor_msgs::MotorData data = getMotorData(motor_id);  // 해당 모터의 데이터를 가져옴

    data.motor_id = motor_id;
    auto now = std::chrono::system_clock::now();
    data.last_update_time = std::chrono::duration<double>(now.time_since_epoch()).count();

    if (pdo_type == PDO_ID::TPDO1) {  // TPDO1 처리 (상태, 에러)
        data.status = update_data.status;
    }
    else if (pdo_type == PDO_ID::TPDO2) {  // TPDO2 처리 (속도, 전류)

        data.feedback_velocity = update_data.feedback_velocity;
        data.encoder = update_data.encoder;
    }
    else if (pdo_type == PDO_ID::TPDO3) {  // TPDO3 처리 (엔코더)
        data.error_code = update_data.error_code;
        data.voltage = update_data.voltage / 1000.0;  // mA -> A 변환
        data.current = update_data.current / 1000.0;  // mA -> A 변환
        if (data.error_code != 0) {
            data.is_error = true;
        }
        else {
            data.is_error = false;
        }
    }
    else if (pdo_type == PDO_ID::RPDO1) {  // TPDO3 처리 (엔코더)
        data.input_velocity = update_data.input_velocity;
    }
    SetMotorData(motor_id, data);
}

void Pcan_driver_samyang_dd::ProcessNmtMessage(int motor_id, const TPCANMsg& o_canmsg)
{
    // NMT 메시지의 첫 번째 바이트는 상태를 나타냅니다.
    int nmt_state = o_canmsg.DATA[0];  // NMT 상태 코드

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

void Pcan_driver_samyang_dd::ResetMotorError(int motor_id)
{
    LOG_INFO("Resetting error for Motor %d", motor_id);

    // 에러 리셋 명령 전송
    PCAN_SAMYANG_DD_RPDO_1 reset_cmd;
    reset_cmd.un16_control_word = uint16_t(0x80);  // 에러 리셋 명령
    Pcan_driver::Write(CanID::CAN_RPDO1_ID + motor_id, reset_cmd.byte_space, 7);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 에러 리셋 후 대기
}

bool Pcan_driver_samyang_dd::IsInitializing()
{
    return !b_init_;
}

PCAN_SAMYANG_DD_SDO_READ Pcan_driver_samyang_dd::SendSDOMsg(int node_id, int command, int index, int sub_index, int data)
{
    PCAN_SAMYANG_DD_SDO_WRITE o_sdo_write;
    o_sdo_write.n8_command = command;
    o_sdo_write.n16_index = index;
    o_sdo_write.n8_subindex = sub_index;
    o_sdo_write.n32_data = data;
    Pcan_driver::Write(CanID::CAN_SDO_REQUEST_ID + node_id, o_sdo_write.byte_space, 8);
    try {
        auto TPCANMsg = sdo_response_manager_.waitForResponse(CanID::CAN_SDO_RESPONSE_ID + node_id, 1000);
        PCAN_SAMYANG_DD_SDO_READ uo_sdo_read;
        memcpy(uo_sdo_read.byte_space, TPCANMsg.DATA, 8);
        return uo_sdo_read;
    }
    catch (std::exception& e) {
        NLOG(error) << "error occurred. " << e.what();
        // return false;
        PCAN_SAMYANG_DD_SDO_READ uo_sdo_read;
        return uo_sdo_read;
    }
}

std::map<int16_t, motor_msgs::MotorData> Pcan_driver_samyang_dd::GetMotorData()
{
    std::lock_guard<std::mutex> lock(data_mutex_);  // 멀티스레드 안전
    return motor_data_;
}