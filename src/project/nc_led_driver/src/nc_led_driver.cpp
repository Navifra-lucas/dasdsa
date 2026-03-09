#include "nc_led_driver.hpp"
using namespace NaviFra;

LedDriver::LedDriver(ros::NodeHandle& nh, ros::NodeHandle& nhp)
    : nh_(nh)
    , nhp_(nhp)
{
    NLOG(info) << "LedDriver Create";

    o_can_driver_.InterfaceOpen();
    RegistListener();
}

LedDriver::~LedDriver()
{
    b_terminate_ = true;
    SendLedCommand(0);
    NLOG(info) << "LedDriver Destroyed";
}

void LedDriver::RegistListener()
{
    sub_led_ = nh_.subscribe("/navifra/led", 5, &LedDriver::SubscribeLedCmd, this);
    sub_bms_info_ = nh_.subscribe("/bms_info", 10, &LedDriver::bmsCallback, this);
    sub_led_docking_ = nh_.subscribe("/docking_led", 5, &LedDriver::dockingledCallback, this);
}

void LedDriver::bmsCallback(const core_msgs::BmsInfo::ConstPtr& msg) {
    // NLOG(info) << "BMS state: " << static_cast<int>(msg->un8_state);
    f_soc_ = msg->f32_soc;
}

void LedDriver::dockingledCallback(const std_msgs::Int64::ConstPtr& msg){
    n_docking_led_ = msg->data;
    if (n_docking_led_ == 1){
        n_front_led_ = 1;  // 전면 도킹
        n_rear_led_ = 0;
    }
    else if(n_docking_led_ == 2){
        n_front_led_ = 0;
        n_rear_led_ = 1;  // 후면 도킹
    }
    else{
        n_front_led_ = 0;
        n_rear_led_ = 0;

    }
}


void LedDriver::SendLedCommand(uint8_t led_status)
{
    can_frame frame;
    frame.can_id = 0xB1;
    frame.can_dlc = 8;
    memset(frame.data, 0, sizeof(frame.data));

    if(led_status == 0){
        frame.data[0] = 0x00;
    }
    else{
        frame.data[0] = 0x01;  // PLC 연결 상태: 1이면 적용
    }
    frame.data[3] = led_status;  // 상태 바이트 위치
    frame.data[4] = static_cast<uint8_t>(f_soc_);  // SOC 값 (0-100)
    frame.data[5] = n_front_led_;
    frame.data[6] = n_rear_led_;


    o_can_driver_.Write(frame);
}

void LedDriver::SubscribeLedCmd(const std_msgs::String::ConstPtr& msg)
{
    const std::string& cmd = msg->data;
    static std::string cmd_pre = "";
    if(cmd != cmd_pre) {
        NLOG(info) << "LED Command Received: " << cmd;
    }

    std::map<std::string, uint8_t> led_map = {
        {"booting",       0},  // 연결 안됨 - 흰색
        {"sw_reset",      1},  // 소프트웨어 리셋 - 분홍생
        {"charging",      5},  // 충전 중/
        {"charged",       6},  // 충전 완료/
        {"standby",       2},  // 연한 파랑/
        {"working",       3},  // 파랑/
        {"pause",         4},  // 파랑 점멸/
        {"emergency",     7},  // 빨강/
        {"error",         8},  // 빨강 점멸/
        {"docking",      10}   // 연한 파랑 점멸
        // {"black",         9},  // 끄기 (reserved)
        // {"white",         6},  // 충전 완료랑 동일 처리
    };

    auto it = led_map.find(cmd);
    if (it != led_map.end()) {
        SendLedCommand(it->second);
    } else {
        NLOG(warning) << "Unknown LED command: " << cmd;
    }
    cmd_pre = cmd;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "nc_led_driver");
    ros::NodeHandle nh, nhp("~");
    NaviFra::LedDriver node(nh, nhp);
    ros::spin();
    return 0;
}
