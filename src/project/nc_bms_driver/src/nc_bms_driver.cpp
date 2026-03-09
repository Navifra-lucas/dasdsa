#include "nc_bms_driver.hpp"

using namespace NaviFra;
BmsDriver::BmsDriver(ros::NodeHandle& nh, ros::NodeHandle& nhp)
    : nh_(nh)
    , nhp_(nhp)
{
    LOG_INFO("BmsDriver Create");
    
    // nh_.param("can/n_can_bus_bms", n_can_bus_bms, 0x37);
    // nh_.param("can/n_can_baud_bms", n_can_baud_bms, 284);

    o_can_driver_.InterfaceOpen();
    o_can_driver_.RegisteCallbackFunc("CanCallback", std::bind(&BmsDriver::CanCallback, this, std::placeholders::_1));

    th_heartbeat_ = std::thread(&BmsDriver::HeartBeatLoop, this);

    ros::param::param<float>("BMS/f_stop_percent", f_stop_percent_, 79.0); // 충전 종료 퍼센트 파라미터
    ros::param::param<float>("BMS/f_resume_percent", f_resume_percent_, 75.0); // 충전 재시작 퍼센트 파라미터 

    ros::param::param<float>("BMS/f_target_linear_x", f_target_linear_x_, 0.01); // cmd_vel 제어 파리미터
    
    RegistTalker();
    RegistListener();
}

BmsDriver::~BmsDriver()
{
    b_terminate_ = true;
    th_heartbeat_.join();
    LOG_INFO("destructor");
}

void BmsDriver::RegistTalker()
{
    pub_bms_ = nh_.advertise<core_msgs::BmsInfo>("/bms_info", 5);
    error_pub_ = nh_.advertise<std_msgs::Int64>("/navifra/error", 5);
    pub_output_cmd_ = nh_.advertise<std_msgs::String>("output_command",5);
}

void BmsDriver::RegistListener()
{
    sub_sleep_cmd_ = nh_.subscribe("/bms_sleep_cmd", 1, &BmsDriver::SleepCmdCallback, this);
    sub_bms_update_cmd_ = nh_.subscribe("/bms_update_cmd", 1, &BmsDriver::UpdateCmdCallback, this);
    sub_bms_log_extract_cmd_ = nh_.subscribe("/bms_log_extract_cmd", 1, &BmsDriver::LogExtractCmdCallback, this);
    sub_output_cmd_ = nh_.subscribe("output_command", 10, &BmsDriver::outputCmdCallback, this);
    sub_navifra_info_ = nh_.subscribe("navifra/info", 10, &BmsDriver::NavifraInfoCallback, this);
    sub_cmd_vel_ = nh_.subscribe("cmd_vel", 10, &BmsDriver::CmdCallback, this);

}

void BmsDriver::HeartBeatLoop()
{
    core_msgs::BmsInfo o_bms_info;
    while (!b_terminate_) {
        {
            std::lock_guard<std::mutex> lock(mtx_info_);
            o_bms_info = o_bms_info_;
        }
        std::vector<uint16_t> request_ids = {
            0x350, 0x351, 0x352, 0x353, 0x354, 0x100, 0x101,
            0x110, 0x111, 0x112, 0x113, 0x114, 0x115
        };
    
        for (auto id : request_ids) {
            uint8_t un8_data[8] = {0};
            un8_data[0] = id & 0xFF;         // LSB
            un8_data[1] = (id >> 8) & 0xFF;  // MSB
    
            o_can_driver_.Write(0x370, un8_data, 8);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        pub_bms_.publish(o_bms_info);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void BmsDriver::outputCmdCallback(const std_msgs::String::ConstPtr& msg)
{
    core_msgs::BmsInfo o_bms_info;
    {
        std::lock_guard<std::mutex> lock(mtx_info_);
        o_bms_info = o_bms_info_;
    }
    if (msg->data == "charge_start" && o_bms_info.f32_soc < f_stop_percent_)
    {
        b_charge_cmd_ = true;
        b_error_check_ = true;
        b_auto_resume_pending_ = false;

        NLOG(info) << "Charging Start Command !! : " << o_bms_info.f32_soc;
    }
    else if (msg->data == "charge_start" && o_bms_info.f32_soc >= f_stop_percent_)
    {
        NLOG(warning) << "Charge command ignored, SOC is already high: " << o_bms_info.f32_soc;
        NLOG(warning) << "Current Battery Percent : " << o_bms_info.f32_soc << " Target Battery Percent : " << f_stop_percent_;
        s_error_.data = core_msgs::NaviAlarm::ERROR_BATTERY_SOC_HIGH;
        // error_pub_.publish(s_error_);
        b_charge_cmd_ = false;
        b_auto_resume_pending_ = true;
        b_manaul_flag_ = true;
        
        std_msgs::String stop_charge_msg;
        stop_charge_msg.data = "charge_stop";
        pub_output_cmd_.publish(stop_charge_msg);
    }
    else if (msg->data == "charge_stop")
    {
        b_charge_cmd_ = false;
    }
}


void BmsDriver::SleepCmdCallback(const std_msgs::UInt8::ConstPtr& msg)
{
    // msg->data : 0 = Wakeup, 1 = Sleep, 2 = Shutdown
    uint8_t un8_data[8] = {0};

    switch (msg->data) {
        case 0: // Wakeup
            un8_data[2] = 0x00;  // Sleep Command Byte (Byte 2)
            break;
        case 1: // Sleep
            un8_data[2] = 0x01;
            break;
        case 2: // Shutdown
            un8_data[3] = 0x01;  // Shutdown Command Byte (Byte 3)
            break;
        default:
            NLOG(warning) << "Invalid Sleep Command received: " << (int)msg->data;
            return;
    }

    o_can_driver_.Write(0x7FE, un8_data, 8);
    NLOG(info) << "Sent SleepCmd [" << (int)msg->data << "] to 0x7FE";
}

void BmsDriver::UpdateCmdCallback(const std_msgs::UInt32::ConstPtr& msg)
{
    // msg->data 상위 16비트 = 명령코드, 하위 16비트 = payload (사이즈 or 오프셋)
    uint16_t cmd = (msg->data >> 16) & 0xFFFF;
    uint16_t payload = msg->data & 0xFFFF;

    uint8_t un8_data[8] = {0};
    un8_data[0] = (cmd >> 8) & 0xFF;   // MSB
    un8_data[1] = cmd & 0xFF;          // LSB
    un8_data[2] = (payload >> 8) & 0xFF;
    un8_data[3] = payload & 0xFF;

    o_can_driver_.Write(0x7F1, un8_data, 8);
    NLOG(info) << "TX bytes: "
           << std::hex << std::setw(2) << std::setfill('0') << (int)un8_data[0] << " "
           << std::hex << std::setw(2) << std::setfill('0') << (int)un8_data[1] << " "
           << std::hex << std::setw(2) << std::setfill('0') << (int)un8_data[2] << " "
           << std::hex << std::setw(2) << std::setfill('0') << (int)un8_data[3];

    NLOG(info) << "Sent UpdateCmd: cmd=0x" << std::hex << cmd << ", payload=0x" << payload;
}

void BmsDriver::LogExtractCmdCallback(const std_msgs::Empty::ConstPtr& msg)
{
    // 0x704: 로그 추출 명령
    uint8_t un8_data[8] = {0};
    un8_data[0] = 0xAA;
    un8_data[2] = 0x55;
    un8_data[4] = 0x33;

    o_can_driver_.Write(0x704, un8_data, 8);
    NLOG(info) << "Sent LogExtractCmd to 0x704";
}


void BmsDriver::CanCallback(const boost::any& any_type_var)
{
    can_frame o_canmsg = boost::any_cast<can_frame>(any_type_var);
    static core_msgs::BmsInfo o_bms_info;
    static std::string s_pre_robot_status = "";
    if(o_canmsg.can_id == 0xA1){
        if(b_charge_cmd_){
            uint8_t un8_data[8] = {0};   
            un8_data[0] = 1;             
            o_can_driver_.Write(0xA0, un8_data, 8);
        }
        else if(b_charge_cmd_ == false){
            uint8_t un8_data[8] = {0};  
            un8_data[0] = 0;
        
            o_can_driver_.Write(0xA0, un8_data, 8);    
        }

        int n_charger_error = o_canmsg.data[0];
        
        if (n_charger_error > 0 && b_error_check_ == true) {
            auto it = charger_error_map_.find(n_charger_error);
            std::string error_msg;

            if (it != charger_error_map_.end()) {
                error_msg = it->second;
            } else {
                error_msg = "알 수 없는 충전기 에러 (" + std::to_string(n_charger_error) + ")";
            }

            s_error_.data = core_msgs::NaviAlarm::ERROR_CHARGER;
            error_pub_.publish(s_error_);

            LOG_INFO("Charger Error: %s", error_msg.c_str());

            std_msgs::String stop_charge_msg;
            stop_charge_msg.data = "charge_stop";
            pub_output_cmd_.publish(stop_charge_msg);

            b_error_check_ = false;
            b_charge_cmd_ = false;
        }
    }
    if(o_canmsg.can_id == 0x350) {
        o_bms_info.b_main_relay        = o_canmsg.data[0];  // Main_Rly
        o_bms_info.b_sleep_request     = o_canmsg.data[1];  // Sleep_Req
        o_bms_info.b_sleep_command     = o_canmsg.data[2];  // Sleep_Cmd
        o_bms_info.b_breaker_off       = o_canmsg.data[3];  // Breaker_Off
    
        uint16_t fw_version = (o_canmsg.data[4] << 8) | o_canmsg.data[5];  // Big Endian
        o_bms_info.n_firmware_version = fw_version;
    
        o_bms_info.b_shutdown_request = o_canmsg.data[6];  // Shutdown_Req
    }    
    
    if(o_canmsg.can_id == 0x351) {
        uint16_t un16_pack_voltage, un16_pack_current, un16_pack_soc, un16_pack_soh = 0;
        un16_pack_voltage = (o_canmsg.data[0] << 8 | o_canmsg.data[1]);
        // un16_pack_current = (o_canmsg.data[2] << 8 | o_canmsg.data[3]);
        int16_t  n16_pack_current  = static_cast<int16_t>(
            (static_cast<uint16_t>(o_canmsg.data[2]) << 8) |
             static_cast<uint16_t>(o_canmsg.data[3]) );

        un16_pack_soc = (o_canmsg.data[4] << 8 | o_canmsg.data[5]);
        un16_pack_soh = (o_canmsg.data[6] << 8 | o_canmsg.data[7]);

        float f_pack_voltage = un16_pack_voltage * 0.01f; // V
        float f_pack_current = n16_pack_current * 0.01f; // A
        float f_pack_soc = un16_pack_soc; // 배터리 몇%인지
        float f_pack_soh = un16_pack_soh; // %

        o_bms_info.f32_pack_volt = f_pack_voltage;
        o_bms_info.f32_pack_current = f_pack_current;
        o_bms_info.f32_soc = f_pack_soc;
        o_bms_info.f32_soh = f_pack_soh;

        if (b_auto_resume_pending_ && f_pack_soc <= f_resume_percent_) {
            if (!b_charge_cmd_) {
                NLOG(info) << "SOC dropped to " << f_pack_soc
                           << "% <= " << f_resume_percent_ << "%. Auto-resume charging.";
                b_charge_cmd_ = true;
    
                std_msgs::String start_charge_msg;
                start_charge_msg.data = "charge_start";
                pub_output_cmd_.publish(start_charge_msg);
            }
            // 한 번 재시작했으면 플래그 해제
            b_auto_resume_pending_ = false;
        }
        // 충전 중인데 SOC가 79% 이상이면 충전 중지
        if (b_charge_cmd_ && o_bms_info.f32_soc >= f_stop_percent_) {
            NLOG(info) << "SOC reached " << o_bms_info.f32_soc
                    << "% >= " << f_stop_percent_ << "%. Stop charging.";

            std_msgs::String stop_charge_msg;
            stop_charge_msg.data = "charge_stop";
            pub_output_cmd_.publish(stop_charge_msg);

            // 자동 재시작 대기 상태로 진입
            b_manaul_flag_ = true;
            b_auto_resume_pending_ = true;
            b_charge_cmd_ = false;
        }

        if(b_auto_resume_pending_)
        {
            if((s_pre_robot_status != s_robot_status_ && s_robot_status_ == "running") || (b_manaul_flag_ && ManualMoveAutoChargeFlag()))
            {
                b_auto_resume_pending_ = false;
                b_manaul_flag_ = false;
                NLOG(info) << "Robot Running ... auto_reset_peding reset ... : " << b_auto_resume_pending_;
            }
        }

        s_pre_robot_status = s_robot_status_;
    }

    if(o_canmsg.can_id == 0x352) { // memcpy
        o_bms_info.n_pov_flag = (o_canmsg.data[0] >> 0) & 0x03;
        o_bms_info.n_puv_flag = (o_canmsg.data[0] >> 2) & 0x03;
        o_bms_info.n_cov_flag = (o_canmsg.data[0] >> 4) & 0x03;
        o_bms_info.n_cuv_flag = (o_canmsg.data[0] >> 6) & 0x03;
        o_bms_info.n_osoc_flag = (o_canmsg.data[1] >> 0) & 0x03;
        o_bms_info.n_usoc_flag = (o_canmsg.data[1] >> 2) & 0x03;
        o_bms_info.n_ot_flag   = (o_canmsg.data[1] >> 4) & 0x03;
        o_bms_info.n_ut_flag   = (o_canmsg.data[1] >> 6) & 0x03;
        o_bms_info.n_odv_flag = (o_canmsg.data[2] >> 0) & 0x03;
        o_bms_info.n_odt_flag = (o_canmsg.data[2] >> 2) & 0x03;
        o_bms_info.n_oc_flag  = (o_canmsg.data[2] >> 4) & 0x03;
        o_bms_info.b_charge_complete = ((o_canmsg.data[2] >> 6) & 0x01) == 1;
        o_bms_info.b_charging   = (o_canmsg.data[3] >> 0) & 0x01;
        if (b_charge_cmd_ == true && o_bms_info.f32_pack_current > 0){
            o_bms_info.b_charging = true;
        }

        if (b_charge_cmd_ == true) {
            if (o_bms_info.f32_pack_current > 0) {
                o_bms_info.b_charging = true;
                n_charge_fail_count_ = 0;   // 성공했으니 카운트 리셋
            }
            else {
                n_charge_fail_count_++;     
                if (n_charge_fail_count_ >=  100) {   // 20번 연속 실패 시 에러
                    s_error_.data = core_msgs::NaviAlarm::ERROR_CHARGING_FAILED;
                    error_pub_.publish(s_error_);
                    n_charge_fail_count_ = 0;  // 에러 후 리셋 (선택사항)
                    b_charge_cmd_ = false; // 충전 실패 시 자동으로 충전 중지
                }
            }
        }
        

        o_bms_info.b_sleep_flag = (o_canmsg.data[3] >> 1) & 0x01;
        o_bms_info.b_relay_fail  = (o_canmsg.data[4] >> 2) & 0x01;
        o_bms_info.b_fuse_fail   = (o_canmsg.data[4] >> 3) & 0x01;
        o_bms_info.b_relay_fusing_fail = (o_canmsg.data[4] >> 4) & 0x01;
        o_bms_info.b_eeprom_fail = (o_canmsg.data[4] >> 5) & 0x01;
        o_bms_info.b_precharge_fail = (o_canmsg.data[4] >> 6) & 0x01;
        o_bms_info.b_AFIC_fail = (o_canmsg.data[4] >> 7) & 0x01;
        o_bms_info.b_ct_mismatch = (o_canmsg.data[5] >> 0) & 0x01;
        o_bms_info.b_pvs_mismatch = (o_canmsg.data[5] >> 1) & 0x01;
        o_bms_info.b_ct_fail = (o_canmsg.data[5] >> 2) & 0x01;
        o_bms_info.b_pvs_fail = (o_canmsg.data[5] >> 3) & 0x01;
        o_bms_info.b_temp_sensor_fail = (o_canmsg.data[5] >> 4) & 0x01;
        o_bms_info.b_cell_volt_sensor_fail = (o_canmsg.data[5] >> 5) & 0x01;
        o_bms_info.b_ct_zeroset_fail = (o_canmsg.data[5] >> 6) & 0x01;
        o_bms_info.b_pvs_zeroset_fail = (o_canmsg.data[5] >> 7) & 0x01;

        
        // 에러 체크 코드
        uint32_t n_bms_error_flags = 0;
        std::vector<std::string> v_error_msgs;
        auto check = [&](bool condition, BmsErrorFlag flag, const std::string& msg) {
            if (condition) {
                n_bms_error_flags |= flag;
                v_error_msgs.push_back(msg);
            }
        };

        check(o_bms_info.n_packovervoltage_flag >= 2, BMS_ERR_PACK_OV, "Pack Over Voltage");
        check(o_bms_info.n_upv_flag >= 2,              BMS_ERR_PACK_UV, "Pack Under Voltage");
        check(o_bms_info.n_ocv_flag >= 2,              BMS_ERR_CELL_OV, "Cell Over Voltage");
        check(o_bms_info.n_ucv_flag >= 2,              BMS_ERR_CELL_UV, "Cell Under Voltage");
        check(o_bms_info.n_osoc_flag >= 2,             BMS_ERR_SOC_OV,  "SOC Over Limit");
        check(o_bms_info.n_usoc_flag >= 2,             BMS_ERR_SOC_UV,  "SOC Under Limit");
        check(o_bms_info.n_ot_flag >= 2,               BMS_ERR_OT,      "Over Temperature");
        check(o_bms_info.n_ut_flag >= 2,               BMS_ERR_UT,      "Under Temperature");
        check(o_bms_info.n_odv_flag >= 2,              BMS_ERR_VOLT_DEV,"Voltage Deviation");
        check(o_bms_info.n_odt_flag >= 2,              BMS_ERR_TEMP_DEV,"Temperature Deviation");
        check(o_bms_info.n_oc_flag >= 2,               BMS_ERR_OC,      "Over Current");

        check(o_bms_info.b_relay_fail,                 BMS_ERR_RELAY,         "Relay Failure");
        check(o_bms_info.b_fuse_fail,                  BMS_ERR_FUSE,          "Fuse Failure");
        check(o_bms_info.b_relay_fusing_fail,          BMS_ERR_RELAY_FUSE,    "Relay Fusing Fail");
        check(o_bms_info.b_eeprom_fail,                BMS_ERR_EEPROM,        "EEPROM Failure");
        check(o_bms_info.b_precharge_fail,             BMS_ERR_PRECHARGE,     "Precharge Failure");
        check(o_bms_info.b_AFIC_fail,                  BMS_ERR_AFIC,          "AFIC Failure");
        check(o_bms_info.b_ct_mismatch,                BMS_ERR_CT_MISMATCH,   "CT Mismatch");
        check(o_bms_info.b_pvs_mismatch,               BMS_ERR_PVS_MISMATCH,  "PVS Mismatch");
        check(o_bms_info.b_ct_fail,                    BMS_ERR_CT_FAIL,       "CT Failure");
        check(o_bms_info.b_pvs_fail,                   BMS_ERR_PVS_FAIL,      "PVS Failure");
        check(o_bms_info.b_temp_sensor_fail,           BMS_ERR_TEMP_SENSOR,   "Temp Sensor Failure");
        check(o_bms_info.b_cell_volt_sensor_fail,      BMS_ERR_CELL_VOLT_SENSOR, "Cell Volt Sensor Failure");
        check(o_bms_info.b_ct_zeroset_fail,            BMS_ERR_CT_ZS_FAIL,    "CT Zero-Set Failure");
        check(o_bms_info.b_pvs_zeroset_fail,           BMS_ERR_PVS_ZS_FAIL,   "PVS Zero-Set Failure");

        if (n_bms_error_flags != 0) {
            s_error_.data = core_msgs::NaviAlarm::ERROR_BMS;
            error_pub_.publish(s_error_);

            std_msgs::String stop_charge_msg;
            stop_charge_msg.data = "charge_stop";
            pub_output_cmd_.publish(stop_charge_msg);
            b_charge_cmd_ = false;

            std::stringstream ss;
            ss << "BMS Error (0x" << std::hex << n_bms_error_flags << "): ";
            for (const auto& msg : v_error_msgs) ss << msg << ", ";
            NLOG(error) << ss.str();
        } else {
            // NLOG(info) << "BMS Status Normal";
        }


    }
    
    if(o_canmsg.can_id == 0x353) {
        uint16_t max_v = (o_canmsg.data[0] << 8) | o_canmsg.data[1];
        uint16_t min_v = (o_canmsg.data[2] << 8) | o_canmsg.data[3];
        uint16_t avg_v = (o_canmsg.data[4] << 8) | o_canmsg.data[5];
    
        o_bms_info.f32_max_cell_v = max_v * 0.0001f;
        o_bms_info.f32_min_cell_v = min_v * 0.0001f;
        o_bms_info.f32_avg_cell_v = avg_v * 0.0001f;
        o_bms_info.un8_max_v_cell_no = o_canmsg.data[6];
        o_bms_info.un8_min_v_cell_no = o_canmsg.data[7];
    }
    
    if(o_canmsg.can_id == 0x354) {
        int8_t max_temp = static_cast<int8_t>(o_canmsg.data[0]);
        int8_t min_temp = static_cast<int8_t>(o_canmsg.data[1]);
        int8_t avg_temp = static_cast<int8_t>(o_canmsg.data[2]);
    
        o_bms_info.f32_max_temp = max_temp;
        o_bms_info.f32_min_temp = min_temp;
        o_bms_info.f32_avg_temp = avg_temp;
        o_bms_info.un8_max_temp_pos = o_canmsg.data[3];
        o_bms_info.un8_min_temp_pos = o_canmsg.data[4];
    }
    
    if(o_canmsg.can_id == 0x100) {
        o_bms_info.f32_cell_voltage[0] = ((o_canmsg.data[0] << 8) | o_canmsg.data[1]) * 0.0001f;
        o_bms_info.f32_cell_voltage[1] = ((o_canmsg.data[2] << 8) | o_canmsg.data[3]) * 0.0001f;
        o_bms_info.f32_cell_voltage[2] = ((o_canmsg.data[4] << 8) | o_canmsg.data[5]) * 0.0001f;
        o_bms_info.f32_cell_voltage[3] = ((o_canmsg.data[6] << 8) | o_canmsg.data[7]) * 0.0001f;
    }
    
    if(o_canmsg.can_id == 0x101) {
        o_bms_info.f32_cell_voltage[4] = ((o_canmsg.data[0] << 8) | o_canmsg.data[1]) * 0.0001f;
        o_bms_info.f32_cell_voltage[5] = ((o_canmsg.data[2] << 8) | o_canmsg.data[3]) * 0.0001f;
        o_bms_info.f32_cell_voltage[6] = ((o_canmsg.data[4] << 8) | o_canmsg.data[5]) * 0.0001f;
        o_bms_info.f32_cell_voltage[7] = ((o_canmsg.data[6] << 8) | o_canmsg.data[7]) * 0.0001f;
    }
    

    if(o_canmsg.can_id == 0x110) {
        o_bms_info.f32_cell_voltage[8]  = ((o_canmsg.data[0] << 8) | o_canmsg.data[1]) * 0.0001f;
        o_bms_info.f32_cell_voltage[9]  = ((o_canmsg.data[2] << 8) | o_canmsg.data[3]) * 0.0001f;
        o_bms_info.f32_cell_voltage[10] = ((o_canmsg.data[4] << 8) | o_canmsg.data[5]) * 0.0001f;
        o_bms_info.f32_cell_voltage[11] = ((o_canmsg.data[6] << 8) | o_canmsg.data[7]) * 0.0001f;
    }
    
    if(o_canmsg.can_id == 0x111) {
        o_bms_info.f32_cell_voltage[12] = ((o_canmsg.data[0] << 8) | o_canmsg.data[1]) * 0.0001f;
        o_bms_info.n_temp[0] = static_cast<int8_t>(o_canmsg.data[2]);
        o_bms_info.n_temp[1] = static_cast<int8_t>(o_canmsg.data[3]);
        o_bms_info.n_temp[2] = static_cast<int8_t>(o_canmsg.data[4]);
        o_bms_info.n_temp[3] = static_cast<int8_t>(o_canmsg.data[5]);
    }
    
    if(o_canmsg.can_id == 0x112) {
        o_bms_info.f32_cell_voltage[13] = ((o_canmsg.data[0] << 8) | o_canmsg.data[1]) * 0.0001f;
        o_bms_info.f32_cell_voltage[14] = ((o_canmsg.data[2] << 8) | o_canmsg.data[3]) * 0.0001f;
        o_bms_info.f32_cell_voltage[15] = ((o_canmsg.data[4] << 8) | o_canmsg.data[5]) * 0.0001f;
        o_bms_info.f32_cell_voltage[16] = ((o_canmsg.data[6] << 8) | o_canmsg.data[7]) * 0.0001f;
    }
    
    if(o_canmsg.can_id == 0x113) {
        o_bms_info.f32_cell_voltage[17] = ((o_canmsg.data[0] << 8) | o_canmsg.data[1]) * 0.0001f;
        o_bms_info.f32_cell_voltage[18] = ((o_canmsg.data[2] << 8) | o_canmsg.data[3]) * 0.0001f;
        o_bms_info.f32_cell_voltage[19] = ((o_canmsg.data[4] << 8) | o_canmsg.data[5]) * 0.0001f;
        o_bms_info.f32_cell_voltage[20] = ((o_canmsg.data[6] << 8) | o_canmsg.data[7]) * 0.0001f;
    }
    
    if(o_canmsg.can_id == 0x114) {
        o_bms_info.f32_cell_voltage[21] = ((o_canmsg.data[0] << 8) | o_canmsg.data[1]) * 0.0001f;
        o_bms_info.f32_cell_voltage[22] = ((o_canmsg.data[2] << 8) | o_canmsg.data[3]) * 0.0001f;
        o_bms_info.f32_cell_voltage[23] = ((o_canmsg.data[4] << 8) | o_canmsg.data[5]) * 0.0001f;
        // 나머지 2바이트는 reserved or unused
    }
    
    if(o_canmsg.can_id == 0x115) {
        for (int i = 0; i < 8; i++) {
            o_bms_info.n_temp[i] = static_cast<int8_t>(o_canmsg.data[i]);
        }
    }
    
    if(o_canmsg.can_id == 0x7FF) {
        NLOG(info) << "[BMS] Received wakeup or sleep or shutdown command from BMS";
    }

    if (o_canmsg.can_id == 0x7F0) {
        uint16_t n_cmd = (o_canmsg.data[1] << 8) | o_canmsg.data[0];  // Byte 0~1: Command Header (Little Endian)
        uint16_t n_result = (o_canmsg.data[3] << 8) | o_canmsg.data[2];  // Byte 2~3: Reserved or result (Optional use)
        uint32_t n_payload =  // Byte 4~7: result or data offset
            (static_cast<uint32_t>(o_canmsg.data[7]) << 24) |
            (static_cast<uint32_t>(o_canmsg.data[6]) << 16) |
            (static_cast<uint32_t>(o_canmsg.data[5]) << 8) |
            (static_cast<uint32_t>(o_canmsg.data[4]));
    
        if (n_cmd == 0x7702) {
            NLOG(info) << "[BMS FW] Requesting data offset: 0x" << std::hex << n_payload;
            // 여기에 대응할 응답 송신 또는 상태 처리 작성
        } else if (n_cmd == 0x7777) {
            NLOG(info) << "[BMS FW] Download result received: code = 0x" << std::hex << n_payload;
            // 성공/실패 여부에 따라 핸들링
        } else {
            NLOG(warning) << "[BMS FW] Unknown CMD: 0x" << std::hex << n_cmd;
        }
    }

    if (o_canmsg.can_id == 0x502) {
        NLOG(info) << "[BMS] Log extraction command received (ID: 0x502)";
    }

    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now - tp_last_log_).count() > 10) {
        NLOG(info) << "BMS Info: "
            // << "cum_volt: " << o_bms_info.f32_cum_volt
            // << ", gather_volt: " << o_bms_info.f32_gather_volt
            // << ", current: " << o_bms_info.f32_current
            << ", soc: " << o_bms_info.f32_soc;
            // << ", state: " << static_cast<int>(o_bms_info.un8_state)
            // << ", bms_life: " << static_cast<int>(o_bms_info.un8_bms_life)
            // << ", remain_cap: " << static_cast<int>(o_bms_info.un32_remain_cap);
        tp_last_log_ = now;
    }

    float f_delta_v_ = o_bms_info.f32_max_cell_v - o_bms_info.f32_min_cell_v;

    if(f_delta_v_ > 0.01f){
        NLOG(warning) << std::fixed << std::setprecision(3)
        << "[max cell volt - min cell volt] " << std::setprecision(0) << f_delta_v_ * 1000 << "mV, "
        << "max_cell=" << o_bms_info.f32_max_cell_v << "V "
        << "min_cell=" << o_bms_info.f32_min_cell_v << "V ";
    }

    {
        std::lock_guard<std::mutex> lock(mtx_info_);
        o_bms_info_ = o_bms_info;
    }
}

void BmsDriver::NavifraInfoCallback(const core_msgs::NavicoreStatus::ConstPtr& msg)
{
    s_robot_status_ = msg->s_status;
}
void BmsDriver::CmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    f_linear_x_ = float(msg->linear.x);
}

bool BmsDriver::ManualMoveAutoChargeFlag()
{
    return (f_linear_x_ >= f_target_linear_x_ && s_robot_status_ == "idle");

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nc_bms_driver");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    BmsDriver nc_bms_driver(nh, nhp);
    ros::spin();

    return 0;
}
